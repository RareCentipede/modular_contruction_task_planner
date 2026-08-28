import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from typing import List, Dict, Any, Optional, Tuple, cast
from trimesh import Trimesh
from yaml import safe_load
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R

from eas.core import Pose, World
from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.block_domain import Object
from modular_construction_task_planner.rbe_solver import compute_stablelego_equilibrium
from modular_construction_task_planner.stability import SupportNode, compute_placement_stability, make_box_mesh, create_support_relation_graph

# ==============================================================================
# 1. CORE STRAIN METRIC (CALCULATION HELPER)
# ==============================================================================

def calculate_overhang_strain(
    pos: list, 
    size: list, 
    goal_config: dict, 
    orientation: Optional[list] = None
) -> float:
    com_x, com_y, com_z = pos[0], pos[1], pos[2]
    half_x, half_y, half_z = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0

    # 1. Ground Level Check (Only flat horizontal base blocks get 0.0 ground strain)
    is_ground = com_z <= half_z + 0.1

    # 2. Robust Tilt Strain Calculation via Matrix Transformed Normal Vector
    tilt_strain = 0.0
    if orientation is not None and len(orientation) in (3, 4):
        try:
            if len(orientation) == 3:
                rot_matrix = R.from_euler('xyz', orientation).as_matrix()
            else:
                rot_matrix = R.from_quat(orientation).as_matrix()

            # Local upward normal vector [0, 0, 1] rotated into world space
            world_up_local = rot_matrix @ np.array([0.0, 0.0, 1.0])
            
            # Dot product with global Z-axis [0, 0, 1]
            cos_theta = np.clip(np.abs(world_up_local[2]), 0.0, 1.0)
            theta_rad = np.arccos(cos_theta)
            
            # Strain ratio = sin(theta): 0.0 at vertical (0 rad), ~0.707 at 45°
            tilt_strain = float(np.sin(theta_rad))
        except Exception:
            tilt_strain = 0.0

    if is_ground and tilt_strain < 1e-3:
        return 0.0

    # 3. Find Supporting Blocks
    supporting_blocks = []
    for other_name, other_data in goal_config.items():
        if 'position' not in other_data:
            continue
        other_pos, other_size = other_data['position'], other_data['size']

        bot_z = com_z - half_z
        top_other_z = other_pos[2] + other_size[2] / 2.0

        if abs(bot_z - top_other_z) < 0.15:
            dx = abs(com_x - other_pos[0])
            dy = abs(com_y - other_pos[1])
            if dx < (half_x + other_size[0] / 2.0) and dy < (half_y + other_size[1] / 2.0):
                supporting_blocks.append((other_pos, other_size))

    # 4. Area Overhang Strain Calculation
    if not supporting_blocks:
        area_strain = 1.0
    else:
        total_overlap_area = 0.0
        block_area = size[0] * size[1]

        for s_pos, s_size in supporting_blocks:
            x_overlap = max(0.0, min(com_x + half_x, s_pos[0] + s_size[0]/2.0) - max(com_x - half_x, s_pos[0] - s_size[0]/2.0))
            y_overlap = max(0.0, min(com_y + half_y, s_pos[1] + s_size[1]/2.0) - max(com_y - half_y, s_pos[1] - s_size[1]/2.0))
            total_overlap_area += (x_overlap * y_overlap)

        unsupported_ratio = 1.0 - min(1.0, total_overlap_area / block_area)
        area_strain = float(np.clip(unsupported_ratio, 0.0, 1.0))

    # Return the maximum of structural bending strain and support coverage deficit
    return float(np.clip(max(area_strain, tilt_strain), 0.0, 1.0))

# ==============================================================================
# 2. CENTRALIZED COMPUTATION ENGINE
# ==============================================================================
def evaluate_obj_stability(world: World, obj: Object, support_graph: Dict[str, SupportNode], ground_mesh: Trimesh,
                           verbose: bool = False) -> Tuple[bool, float]:
    """
        Evaluate the stability of the branch by checking the support score of the target block after placing.
        If the score is below the threshold, return a high cost to discourage exploring this branch.

        Returns a tuple of whether the placement is stable and the support score.
    """
    branch_support_score = 0.0
    obj_support_node = support_graph[obj.name]
    supporting_objs = []
    supp_names = []
    sd = {}

    for parent_name, (score, is_placed) in obj_support_node.supporting_objects.items():
        if is_placed:
            branch_support_score += score
            parent_obj = world.entities.get_entities(parent_name)
            parent_obj = cast(Object, parent_obj)
            supporting_objs.append(parent_obj)
            supp_names.append(parent_name)

    is_stable = branch_support_score >= obj_support_node.support_threshold

    if not is_stable:
        overall_support_score = obj_support_node.support_combo_dict.get(tuple(supp_names), None)
        branch_support_score = overall_support_score if overall_support_score else branch_support_score
        if not overall_support_score and supp_names:
            sd, overall_support_score = compute_placement_stability(obj, supporting_objs, [], ground_mesh, verbose)
            branch_support_score = overall_support_score
            obj_support_node.support_combo_dict.update({tuple(supp_names): overall_support_score})

    if verbose:
        print(f"Evaluating stability for object {obj.name} with supporting objects {supp_names}. Initial check: {'passed' if is_stable else 'not passed'}\n"
                f"Individual support scores: {[supp_name + ': ' + str(score) for supp_name, (score, _) in support_graph[obj.name].supporting_objects.items()]}.\n"
                f"Combined support score: {branch_support_score}. Threshold: {obj_support_node.support_threshold}.")
        print(f"Support data: {sd}")

    is_stable = branch_support_score >= obj_support_node.support_threshold
    return is_stable, branch_support_score

def compute_construction_metrics(
    problem_name: str,
    problem_config_path: str = "configs/problem_configs/",
    construction_sequence: Optional[List[str]] = None
) -> Dict[str, Any]:
    """
        Executes all physics computations, LP equilibrium checks, CoM evaluations,
        and sequence step strain evaluations in a single pass.
    """
    world = parse_configs_to_world(problem_name, problem_config_path)
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))

    all_objects = cast(List[Object], world.entities.get_entities(Object))
    obj_dict = {obj.name: obj for obj in all_objects}

    # Set up static state for global calculations
    for obj in all_objects:
        if obj.goal.value:
            obj.at.value = obj.goal.value

    # 1. Full-Structure Heatmap & Equilibrium
    is_full_stable, full_residuals, full_contact_forces = compute_stablelego_equilibrium(all_objects, world.pose_dict)
    block_strains = {}
    block_meshes = {}

    for name, data in goal_config.items():
        obj = world.entities.get_entities(name)
        obj = cast(Object, obj)
        goal_pose = world.pose_dict.get(obj.goal.value) if obj.goal.value else None

        if goal_pose:
            pos, size = data['position'], data['size']
            obj.at.value = obj.goal.value
            block_strains[name] = calculate_overhang_strain(pos, size, goal_config, data['orientation'])
            block_meshes[name] = make_box_mesh(size, goal_pose.homogeneous)

    # 2. Ground Hull & Global Center of Mass (CoM)
    ground_points = []
    total_mass = 0.0
    weighted_com = np.zeros(2)

    for name, data in goal_config.items():
        if 'position' not in data:
            continue
        pos = np.array(data['position'])
        size = np.array(data['size'])
        mass = data.get('mass', 1.0)

        weighted_com += pos[:2] * mass
        total_mass += mass

        if abs(pos[2] - size[2] / 2.0) < 1e-2:
            hx, hy = size[0] / 2.0, size[1] / 2.0
            ground_points.extend([
                [pos[0] - hx, pos[1] - hy],
                [pos[0] + hx, pos[1] - hy],
                [pos[0] + hx, pos[1] + hy],
                [pos[0] - hx, pos[1] + hy],
            ])

    global_com = weighted_com / total_mass if total_mass > 0 else np.zeros(2)
    ground_pts = np.array(ground_points)
    hull_simplices = []
    hull_vertices = []

    if len(ground_pts) >= 3:
        hull = ConvexHull(ground_pts)
        hull_simplices = hull.simplices
        hull_vertices = hull.vertices

    # 3. Step-by-Step Sequence Evaluation
    if construction_sequence is None:
        construction_sequence = [k for k in goal_config.keys() if 'position' in goal_config[k]]
        construction_sequence.sort(key=lambda k: goal_config[k]['position'][2])

    steps = []
    stability_statuses = []
    max_strains = []
    mean_strains = []
    latest_strains = []

    active_config = {}
    active_objects = []

    # Inside compute_construction_metrics step-by-step loop:
    net_forces = []
    net_torques = []
    support_scores = []
    block_support_scores = {}

    ground_mesh, support_graph = create_support_relation_graph(world, support_ratio_threshold=0.7)

    for idx, block_name in enumerate(construction_sequence, start=1):
        if block_name in goal_config:
            active_config[block_name] = goal_config[block_name]

        if block_name in obj_dict:
            obj = obj_dict[block_name]
            if obj.goal.value:
                obj.at.value = obj.goal.value
            active_objects.append(obj)

        # 1. Unpack 3-tuple from updated rbe_solver
        is_stable, net_force, net_torque = compute_stablelego_equilibrium(active_objects, world.pose_dict)

        current_strains = [
            calculate_overhang_strain(data['position'], data['size'], active_config, data['orientation']) 
            for data in active_config.values() if 'position' in data
        ]

        support_score = evaluate_obj_stability(world, obj, support_graph, ground_mesh)[1]

        obj_support_node = support_graph[block_name]
        obj_support_node.current_support_score = support_score

        for supported_name, supported_edge in obj_support_node.supported_objects.items():
            obj_support_node.supported_objects[supported_name] = (supported_edge[0], True)
            supported_obj_support_node = support_graph[supported_name]
            supported_obj_support_node.supporting_objects[block_name] = (supported_edge[0], True)

        steps.append(idx)
        stability_statuses.append(is_stable)

        net_forces.append(net_force)
        net_torques.append(net_torque)
        max_strains.append(max(current_strains) if current_strains else 0.0)
        mean_strains.append(float(np.mean(current_strains)) if current_strains else 0.0)
        latest_strains.append(current_strains[-1] if current_strains else 0.0)
        support_scores.append(support_score)
        block_support_scores[block_name] = support_score

    # Return dictionary updated with max_contact_forces
    return {
        "problem_name": problem_name,
        "goal_config": goal_config,
        "is_full_stable": is_full_stable,
        "block_strains": block_strains,
        "block_meshes": block_meshes,
        "ground_pts": ground_pts,
        "hull_simplices": hull_simplices,
        "hull_vertices": hull_vertices,
        "global_com": global_com,
        "construction_sequence": construction_sequence,
        "steps": steps,
        "stability_statuses": stability_statuses,
        "net_forces": net_forces,
        "net_torques": net_torques,
        "max_strains": max_strains,
        "mean_strains": mean_strains,
        "latest_strains": latest_strains,
        "support_scores": support_scores,
        "block_support_scores": block_support_scores
    }

# ==============================================================================
# 3. PURE PLOTTING FUNCTIONS
# ==============================================================================

def plot_friction_heatmap(data: Dict[str, Any], show: bool = False):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f"Cantilever Moment Strain Heatmap: {data['problem_name']}\n(Structure Stable: {data['is_full_stable']})", fontsize=13, fontweight='bold')

    cmap = cm.get_cmap('YlOrRd')
    norm = mcolors.Normalize(vmin=0.0, vmax=1.0)
    all_verts = []

    for name, block_data in data['goal_config'].items():
        if 'position' not in block_data:
            continue

        pos, size = block_data['position'], block_data['size']
        mesh = data['block_meshes'][name]
        all_verts.append(mesh.vertices)

        strain = 1-data['block_support_scores'][name]
        rgb_color = cmap(strain)[:3]

        tris = mesh.vertices[mesh.faces]
        poly = Poly3DCollection(tris, alpha=0.5)
        poly.set_facecolor(rgb_color)
        poly.set_edgecolor('black')
        poly.set_linewidth(1.0)
        ax.add_collection3d(poly)

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('Inverse support score', fontweight='bold')

    flat = np.vstack(all_verts)
    max_range = (flat.max(axis=0) - flat.min(axis=0)).max() / 2.0
    mid = (flat.max(axis=0) + flat.min(axis=0)) / 2.0
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(0, mid[2] + max_range)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    plt.tight_layout()
    if show:
        plt.show()

    if SAVE:
        plt.savefig(f"~/Documents/Thesis/final_paper/{data['problem_name']}/{data['problem_name']}_strain_heatmap.pdf", dpi=300)
        print(f"Saved strain heatmap to ~/Documents/Thesis/final_paper/{data['problem_name']}_strain_heatmap.pdf")

def plot_construction_sequence_strain(data: Dict[str, Any], show: bool = False):
    steps = data['steps']
    sequence = data['construction_sequence']

    plt.figure(figsize=(9, 5))
    plt.plot(steps, data['max_strains'], color='#d95f02', marker='o', linewidth=2.5, label='Peak Overhang Strain Ratio')
    plt.plot(steps, data['mean_strains'], color='#7570b3', linestyle='--', marker='s', linewidth=1.5, label='Average Structure Strain Ratio')
    plt.plot(steps, data['latest_strains'], color='#1b9e77', linestyle=':', marker='^', linewidth=1.5, label='Latest Block Strain Ratio')

    plt.axhline(y=0.7, color='r', linestyle=':', label='High Strain Warning Threshold (0.7)')
    plt.title(f"Construction Sequence Strain Evolution: {data['problem_name']}", fontsize=13, fontweight='bold', pad=12)
    plt.xlabel("Construction Step (Block Placed)", fontweight='bold')
    plt.ylabel("Strain Ratio [0.0 = Stable, 1.0 = Max Moment]", fontweight='bold')
    plt.xticks(steps, [f"Step {s}\n({sequence[s-1]})" for s in steps], rotation=30, ha='right', fontsize=9)
    plt.ylim(-0.05, 1.05)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper left')
    plt.tight_layout()

    if show:
        plt.show()

def plot_com_projection(data: Dict[str, Any], show: bool = False):
    ground_pts = data['ground_pts']
    global_com = data['global_com']

    plt.figure(figsize=(7, 7))
    plt.title(f"Center of Mass Projection vs Ground Hull\n({data['problem_name']})", fontsize=13, fontweight='bold')

    if len(ground_pts) >= 3:
        for simplex in data['hull_simplices']:
            plt.plot(ground_pts[simplex, 0], ground_pts[simplex, 1], 'k-', linewidth=2.0)
        plt.fill(ground_pts[data['hull_vertices'], 0], ground_pts[data['hull_vertices'], 1], color='lightgray', alpha=0.5, label='Base Support Polygon')

    for name, block_data in data['goal_config'].items():
        if 'position' in block_data:
            pos = block_data['position']
            plt.scatter(pos[0], pos[1], c='blue', s=30, alpha=0.6)
            plt.text(pos[0] + 0.05, pos[1] + 0.05, name, fontsize=8)

    plt.scatter(global_com[0], global_com[1], c='red', s=180, marker='X', zorder=10, label=f'Global CoM ({global_com[0]:.2f}, {global_com[1]:.2f})')

    plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.xlabel('X Coordinate [m]')
    plt.ylabel('Y Coordinate [m]')
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    plt.tight_layout()

    if show:
        plt.show()

def plot_construction_force_and_strain(data: Dict[str, Any], show: bool = False):
    steps = data['steps']
    sequence = data['construction_sequence']
    colors = ['#2ca02c' if stable else '#d62728' for stable in data['stability_statuses']]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    legend_elements_contact = [
        Line2D([0], [0], color='#1f77b4', lw=2, label='Net Force per step [N]'),
        Line2D([0], [0], color='#ff7f0e', lw=2, linestyle='--', label='Net Torque per step [N·m]'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#2ca02c', markersize=9, label='Equilibrium Stable'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#d62728', markersize=9, label='Unstable Step')
    ]

    ax1.plot(steps, data['net_forces'], color='#1f77b4', linestyle='-', linewidth=2, label='Net Force per step [N]')
    ax1.plot(steps, data['net_torques'], color='#ff7f0e', linestyle='--', linewidth=2, label='Net Torque steps [N·m]')
    for s, force, torque, c in zip(steps, data['net_forces'], data['net_torques'], colors):
        ax1.scatter(s, force, color=c, s=80, zorder=5)
        ax1.scatter(s, torque, color=c, s=80, zorder=5)

    ax1.set_title(f"Construction Phase Stability Analysis: {data['problem_name']}", fontsize=13, fontweight='bold', pad=12)
    ax1.set_ylabel("Max Normal Reaction Force [N]", fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(handles=legend_elements_contact, loc='upper left')

    # ax2.plot(steps, data['max_strains'], color='#d95f02', marker='s', linewidth=2, label='Max Overhang Strain Ratio')
    ax2.plot(steps, data['support_scores'], color='#9467bd', marker='^', linestyle='--', linewidth=1.5, label='Support Score')
    ax2.axhline(y=0.7, color='r', linestyle=':', label='Stability Threshold (0.7)')
    ax2.set_xlabel("Construction Step (Block Placed)", fontweight='bold')
    ax2.set_ylabel("Max Strain Ratio [0.0 - 1.0]", fontweight='bold')
    ax2.set_ylim(-0.05, 1.05)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower left')

    plt.xticks(steps, [f"Step {s}\n({sequence[s-1]})" for s in steps], rotation=25, ha='right')
    plt.tight_layout()

    if show:
        plt.show()

    if SAVE:
        plt.savefig(f"~/Documents/Thesis/final_paper/{data['problem_name']}/{data['problem_name']}_phase_stability_analysis.pdf", dpi=300)
        print(f"Saved phase stability analysis to ~/Documents/Thesis/final_paper/{data['problem_name']}_phase_stability_analysis.pdf")

# ==============================================================================
# 4. ENTRY POINT
# ==============================================================================
SAVE = False
if __name__ == "__main__":
    problem_name = "shifted_tower"

    # Compute ALL data once
    computed_data = compute_construction_metrics(problem_name)

    # Plot without re-calculating physics/geometry
    plot_friction_heatmap(computed_data)
    plot_construction_sequence_strain(computed_data)
    plot_com_projection(computed_data)
    plot_construction_force_and_strain(computed_data)

    plt.show()