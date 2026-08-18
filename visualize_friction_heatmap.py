import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import List, Dict, cast
from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.block_domain import Object
from modular_construction_task_planner.rbe_solver import compute_stablelego_equilibrium
from modular_construction_task_planner.stability import make_box_mesh

def calculate_overhang_strain(pos: list, size: list, goal_config: dict) -> float:
    com_x, com_y, com_z = pos[0], pos[1], pos[2]
    half_x, half_y, half_z = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0

    # 1. Base blocks touching ground always have zero cantilever strain
    if com_z <= half_z + 0.1:
        return 0.0

    supporting_blocks = []

    # 2. Relaxed surface alignment search
    for other_name, other_data in goal_config.items():
        if 'position' not in other_data:
            continue
        other_pos = other_data['position']
        other_size = other_data['size']

        # Check if top of lower block aligns with bottom of current block (larger tolerance)
        bot_z = com_z - half_z
        top_other_z = other_pos[2] + other_size[2] / 2.0

        if abs(bot_z - top_other_z) < 0.1:
            dx = abs(com_x - other_pos[0])
            dy = abs(com_y - other_pos[1])

            # Check bounding box XY overlap
            if dx < (half_x + other_size[0] / 2.0) and dy < (half_y + other_size[1] / 2.0):
                supporting_blocks.append((other_pos, other_size))

    # 3. Unsupported floating block
    if not supporting_blocks:
        return 1.0

    # 4. Compute weighted support area overlap
    total_overlap_area = 0.0
    block_area = size[0] * size[1]

    for s_pos, s_size in supporting_blocks:
        x_overlap = max(0.0, min(com_x + half_x, s_pos[0] + s_size[0]/2.0) - max(com_x - half_x, s_pos[0] - s_size[0]/2.0))
        y_overlap = max(0.0, min(com_y + half_y, s_pos[1] + s_size[1]/2.0) - max(com_y - half_y, s_pos[1] - s_size[1]/2.0))
        total_overlap_area += (x_overlap * y_overlap)

    # Strain reflects the fraction of the block's base left unsupported in mid-air
    unsupported_ratio = 1.0 - min(1.0, total_overlap_area / block_area)
    return float(np.clip(unsupported_ratio, 0.0, 1.0))

def plot_friction_heatmap(problem_name: str = "cantilever_bridge", problem_config_path: str = "configs/problem_configs/"):
    world = parse_configs_to_world(problem_name, problem_config_path)
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    
    objects = world.entities.get_entities(Object)
    objects = cast(List[Object], objects)  # Type hint for clarity
    for obj in objects:
        if obj.goal.value:
            obj.at.value = obj.goal.value

    is_stable, residuals = compute_stablelego_equilibrium(objects, world.pose_dict, default_mass=1000.0)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f"Cantilever Moment Strain Heatmap: {problem_name}\n(Structure Stable: {is_stable})", fontsize=13, fontweight='bold')

    cmap = cm.get_cmap('YlOrRd')  # Yellow (low strain) -> Orange -> Red (high strain)
    norm = mcolors.Normalize(vmin=0.0, vmax=1.0)
    all_verts = []

    for name, data in goal_config.items():
        if 'position' not in data:
            continue
        
        pos, size = data['position'], data['size']
        mesh = make_box_mesh(size, pos)
        all_verts.append(mesh.vertices)

        # Compute localized strain / moment stress
        strain = calculate_overhang_strain(pos, size, goal_config)
        rgb_color = cmap(norm(strain))[:3]

        # Draw semi-transparent block
        tris = mesh.vertices[mesh.faces]
        poly = Poly3DCollection(tris, alpha=0.5)
        poly.set_facecolor(rgb_color)
        poly.set_edgecolor('black')
        poly.set_linewidth(1.0)
        ax.add_collection3d(poly)

        # Draw high-contrast wireframe to highlight overhanging sections
        hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
        x_lines = np.linspace(pos[0] - hx, pos[0] + hx, 3)
        y_lines = np.linspace(pos[1] - hy, pos[1] + hy, 3)
        z_lines = np.linspace(pos[2] - hz, pos[2] + hz, 3)

        for x in x_lines:
            for y in y_lines:
                ax.plot([x, x], [y, y], [pos[2] - hz, pos[2] + hz], color=rgb_color, alpha=0.9, linewidth=1.5)
        for x in x_lines:
            for z in z_lines:
                ax.plot([x, x], [pos[1] - hy, pos[1] + hy], [z, z], color=rgb_color, alpha=0.9, linewidth=1.5)

    # Colorbar Legend
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('Overhang Bending Strain / Torque Ratio', fontweight='bold')

    # View bounds
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
    plt.show()

if __name__ == "__main__":
    plot_friction_heatmap("temple_facade")