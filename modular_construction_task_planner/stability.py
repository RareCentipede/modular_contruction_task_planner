import trimesh
import colorsys
import random
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx

from scipy.spatial import ConvexHull
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh import Trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from typing import Dict, List, Tuple, cast, Optional
from dataclasses import dataclass, field

from eas.core import World
from modular_construction_task_planner.block_domain import Object

@dataclass
class SupportNode:
    name: str
    area: float
    support_threshold: float
    total_support_ratio: float = 0.0
    current_support_score: float = 0.0
    supporting_objects: Dict[str, Tuple[float, bool]] = field(default_factory=dict) # Objects that support THIS object.
    # (name, support_score, is_placed)
    supported_objects: Dict[str, Tuple[float, bool]] = field(default_factory=dict) # Objects that this object SUPPORTS
    support_combo_dict: Dict[Tuple[str, ...], float] = field(default_factory=dict) # For future use: mapping of specific combinations of supporting objects to their combined support score.

    @property
    def supported(self) -> bool:
        return self.current_support_score >= self.support_threshold

    def __str__(self):
        return (f"SupportNode(name={self.name}, area={self.area:.2f}, "
                f"current_support_score={self.current_support_score:.2f}, "
                f"support_threshold={self.support_threshold:.2f}, "
                f"supported={self.supported}, "
                f"supporting_objects={self.supporting_objects}, "
                f"supported_objects={self.supported_objects}), "
                f"support_combo_dict={self.support_combo_dict})")

# Assume cubes
def create_support_relation_graph(world: World,
                                  support_ratio_threshold: float = 0.7,
                                  verbose: bool = False) -> Tuple[Trimesh, Dict[str, SupportNode]]:
    """
        Create a support relation graph based on the current world state.
        Each node represents an object, and edges represent support relationships.
        The area of support is calculated based on the contact area between objects.
    """
    support_graph = {}
    goal_pose_dict = {}

    objs = world.entities.get_entities(Object)
    objs = cast(List[Object], objs)
    obj_with_goals = np.array([obj for obj in objs if obj.goal.value])
    for obj in obj_with_goals:
        goal_pose = world.pose_dict.get(obj.goal.value) # type: ignore
        goal_pose_dict[obj.name] = goal_pose
        # goal_pose.position[2] += obj.dim[2] / 2 # type: ignore
        obj.mesh
        obj.mesh.apply_transform(goal_pose.homogeneous) # type: ignore

    goal_poses = np.array([goal_pose.position for goal_pose in goal_pose_dict.values()])
    goal_positions = np.array([[g[0], g[1], g[2]] for g in goal_poses])
    # Create a ground plane at z=0
    max_x = np.max(goal_positions[:, 0]) + 1.0
    max_y = np.max(goal_positions[:, 1]) + 1.0
    min_x = np.min(goal_positions[:, 0]) - 1.0
    min_y = np.min(goal_positions[:, 1]) - 1.0
    ground_vertices = np.array([[min_x, min_y, 0], [max_x, min_y, 0], [max_x, max_y, 0], [min_x, max_y, 0]])
    ground_faces = np.array([[0, 1, 2], [0, 2, 3]])
    ground_mesh = Trimesh(vertices=ground_vertices, faces=ground_faces)

    for i, obj in enumerate(obj_with_goals):
        goal_pos = goal_positions[i]
        candidate_position_idx = goal_positions[:, 2] < goal_pos[2] # Objects below the current object
        candidate_objs = obj_with_goals[candidate_position_idx]

        support_data, overall_support_area_ratio = compute_placement_stability(obj,
                                                                               candidate_objs, # type: ignore
                                                                               [],
                                                                               ground_mesh,
                                                                               verbose=verbose)
        supporting_objs = {name: (score, False) for name, score in support_data.items() if name != 'area' and name != 'g'}
        support_node = SupportNode(
            name=obj.name,
            area=support_data['area'],
            total_support_ratio=overall_support_area_ratio,
            support_threshold=support_ratio_threshold,
            supporting_objects=supporting_objs
        )
        support_graph[obj.name] = support_node
        if verbose:
            print(f"Object '{obj.name}' support data: {support_data}, overall support ratio: {overall_support_area_ratio:.2f}")

        if 'g' in support_data:
            support_node.supporting_objects['g'] = (support_data['g'], True) # Ground support is considered already placed
            support_node.current_support_score += support_data['g']

    for support_node in support_graph.values():
        for name, (score, _) in support_node.supporting_objects.items():
            if name in support_graph:
                support_graph[name].supported_objects[support_node.name] = (score, False)

    return ground_mesh, support_graph

def compute_placement_stability(object: Object | Trimesh,
                                candidate_support_objs: List[Object] | List[Trimesh],
                                supp_names: List[str],
                                ground_plane: Trimesh,
                                verbose: bool = False) -> Tuple[Dict[str, float], float]:
    """
        Compute the stability of placing the object at its goal position based on support area analysis.
    """
    support_data = {}
    support_polys = []
    all_contact_vertices = []
    overall_support_area_ratio = 0.0

    # Handle object type extraction
    mesh_above = object.mesh if isinstance(object, Object) else object
    footprint = Polygon(mesh_above.vertices[:, :2]).convex_hull
    support_data['area'] = footprint.area

    # 1. Ground contact check
    ground_contact_polygon = get_contact_polygon(mesh_above, ground_plane)
    if ground_contact_polygon and not ground_contact_polygon.is_empty:
        support_data['g'] = 1.0  # Ground provides full support
        overall_support_area_ratio = ground_contact_polygon.area / footprint.area
        return support_data, overall_support_area_ratio

    # 2. Candidate objects check
    for i, supp in enumerate(candidate_support_objs):
        mesh_below = supp.mesh if isinstance(supp, Object) else supp
        supp_name = supp.name if isinstance(supp, Object) else supp_names[i]

        contact_polygon = get_contact_polygon(mesh_above, mesh_below)
        if not contact_polygon or contact_polygon.is_empty:
            continue

        minx, miny, maxx, maxy = contact_polygon.bounds
        all_contact_vertices.extend([(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)])
        support_polygon = contact_polygon.convex_hull
        support_score = support_polygon.area / footprint.area
        support_data[supp_name] = support_score
        support_polys.append(support_polygon)

        if verbose:
            print(f"\nObject '{object.name if isinstance(object, Object) else 'unknown'}' support from '{supp_name}':")
            print(f"Support from '{supp_name}': contact area = {support_polygon.area:.2f}, "
                  f"support score = {support_score:.2f}")
            print(f"Object vertices: {[b for b in footprint.bounds]},\n"
                  f"Support object vertices: {[b for b in mesh_below.bounds]},\n"
                  f"Support polygon vertices: {[(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]},\n")

    if len(all_contact_vertices) > 4:
        global_hull = Polygon(all_contact_vertices).convex_hull
        overall_support_area_ratio = global_hull.area / footprint.area
        if verbose:
            print(f"Overall support area ratio from combined supports: {overall_support_area_ratio:.2f}\n"
                  f"Global support polygon vertices: {all_contact_vertices},\n"
                  f"Global hull area: {global_hull.area:.2f}, Footprint area: {footprint.area:.2f}")

    return support_data, overall_support_area_ratio

def get_contact_polygon(mesh_above: Trimesh, mesh_below: Trimesh, contact_z_tolerance: float = 0.05):
    """
    Finds the contact polygon in the XY plane by projecting overlapping 
    features, resolving floating-point vulnerabilities found in direct slicing.
    """
    # Verify vertical proximity
    z_above_min = mesh_above.bounds[0][2]
    z_below_max = mesh_below.bounds[1][2]
    
    gap = abs(z_above_min - z_below_max)
    if gap > contact_z_tolerance:
        return None

    # Get 2D footprints of both items in the XY plane
    poly_above = Polygon(mesh_above.vertices[:, :2]).convex_hull
    poly_below = Polygon(mesh_below.vertices[:, :2]).convex_hull

    # Calculate their overlapping footprint 
    intersection_poly = poly_above.intersection(poly_below)
    
    if intersection_poly.is_empty:
        return None

    return intersection_poly

def make_box_mesh(size: list, position: list) -> trimesh.Trimesh:
    """Helper to generate a transformed Trimesh box given size and position."""
    mesh = trimesh.creation.box(extents=size)
    # The size[2] / 2 translation is skipped if your position coordinates 
    # are already absolute centers rather than bottom-centers.
    mesh.apply_translation(position)
    return mesh

def visualize_goal_structure(goal_data: dict, unique_colors: List[Tuple[float, float, float]],
                             title: str = "Target Goal Structure Layout", show: bool = True):
    """
        Visualizes the parsed goal.yaml structure in 3D.
        
        Args:
            goal_data: A dictionary mapping block names to their properties:
                    { 'block1': {'position': [...], 'size': [l, w, h], 'color': [...]}, ... }
            title: Title of the generated plot
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title, fontsize=14, fontweight='bold', pad=20)

    all_vertices = []
    # Track unique colors for the legend if needed
    legends = []

    # Iterate and draw each block
    for (name, data), random_color in zip(goal_data.items(), unique_colors):
        if name == 'robot' or 'position' not in data:
            continue

        patch = mpatches.Patch(color=random_color, label=name, alpha=0.8)
        legends.append(patch)

        pos = data['position']
        size = data['size']
        # Default to a nice slate blue if color isn't provided in the goal dict
        color = data.get('color', random_color) 

        # Ensure alpha/transparency channel is attached for clear overlay overlapping
        if len(color) == 3:
            color = list(color) + [0.6]

        # Generate mesh representation
        mesh = make_box_mesh(size, pos)
        all_vertices.append(mesh.vertices)

        # Convert Trimesh faces into matplotlib Poly3DCollection elements
        tris = mesh.vertices[mesh.faces]
        poly3d = Poly3DCollection(tris, alpha=color[3])
        poly3d.set_facecolor(color[:3])
        poly3d.set_edgecolor([0.1, 0.1, 0.1]) # Crisp dark edges
        poly3d.set_linewidth(0.6)

        ax.add_collection3d(poly3d)

    # Compile vertices to set appropriate axis bounds dynamically
    if all_vertices:
        flat_verts = np.vstack(all_vertices)
        max_x, min_x = flat_verts[:, 0].max(), flat_verts[:, 0].min()
        max_y, min_y = flat_verts[:, 1].max(), flat_verts[:, 1].min()
        max_z = flat_verts[:, 2].max()

        margin = 1.0
        lim = max(max_x - min_x, max_y - min_y, max_z) / 2 + margin
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(0, max_z + margin)

        # Draw an explicit light grey ground grid at Z = 0
        extent_x = max(abs(min_x), abs(max_x)) + margin
        extent_y = max(abs(min_y), abs(max_y)) + margin
        gx, gy = np.meshgrid(np.linspace(-extent_x, extent_x, 10), np.linspace(-extent_y, extent_y, 10))
        gz = np.zeros_like(gx)
        ax.plot_wireframe(gx, gy, gz, color=(0.7, 0.7, 0.7, 0.2), linewidth=0.8)
    else:
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_zlim(0, 5)

    # Clean up axes layouts
    ax.set_xlabel('X (Width)', fontweight='bold')
    ax.set_ylabel('Y (Depth)', fontweight='bold')
    ax.set_zlabel('Z (Height)', fontweight='bold')

    # Initial camera view angle (isometric-leaning bird's-eye perspective)
    ax.view_init(elev=22, azim=-55)

    # ax.legend(
    #     handles=legends,
    #     loc='upper left',
    #     bbox_to_anchor=(1.05, 1), # Moves the legend just to the right of the plot area
    #     borderaxespad=0.,
    #     title="Objects List",
    #     title_fontproperties={'weight': 'bold'}
    # )

    plt.tight_layout()
    if show:
        plt.show()

def visualize_support_node_graph(support_graph: Dict[str, SupportNode],
                                 goal_data: Optional[Dict] = None,
                                 colors: List[List[float]] = [],
                                 title: str = "Structural Support & Stability Graph",
                                 show: bool = True):
    """
        Visualizes the support relations using a structured hierarchical layout.
    """
    G = nx.DiGraph()
    labels = {}

    # 1. Build nodes and default labels
    for name, node in support_graph.items():
        G.add_node(name)
        labels[name] = f"{name}"

    # Default colors if not provided
    if not colors:
        colors = []
        for name, node in support_graph.items():
            if node.supported:
                colors.append([0.2, 0.65, 0.3])  # Stable Green
            else:
                colors.append([0.85, 0.3, 0.2])  # Unstable Red

    # 2. Extract directed edges (Parent/Supporting -> Child/Supported)
    for name, node in support_graph.items():
        for parent_name, (score, _) in node.supporting_objects.items():
            if parent_name in support_graph:
                G.add_edge(parent_name, name, weight=score)

    # 3. Compute Deterministic Hierarchical Layer Layout (Bottom-Up)
    # Determine the depth/level of each node
    levels: Dict[str, int] = {}
    
    # Base nodes (supported by ground 'g' or no parents in the object graph)
    for node_name in G.nodes():
        parents = list(G.predecessors(node_name))
        if not parents:
            levels[node_name] = 0

    # Propagate levels upward using longest path calculation
    changed = True
    while changed:
        changed = False
        for node_name in G.nodes():
            parents = list(G.predecessors(node_name))
            if parents:
                max_parent_level = max(levels.get(p, 0) for p in parents)
                new_level = max_parent_level + 1
                if levels.get(node_name) != new_level:
                    levels[node_name] = new_level
                    changed = True

    # Group nodes by their calculated hierarchy level
    level_groups: Dict[int, List[str]] = {}
    for node_name, lvl in levels.items():
        level_groups.setdefault(lvl, []).append(node_name)

    # Assign explicit (x, y) coordinates for a structured tree layout
    pos_layout = {}
    for lvl, nodes_in_lvl in level_groups.items():
        num_nodes = len(nodes_in_lvl)
        # Evenly space nodes horizontally across x-axis for each tier level
        x_coords = np.linspace(-1.0, 1.0, num_nodes) if num_nodes > 1 else [0.0]
        for idx, node_name in enumerate(nodes_in_lvl):
            pos_layout[node_name] = (x_coords[idx], float(lvl))

    # 4. Render Graph Elements
    plt.figure(figsize=(7, 5))
    plt.title(title, fontsize=14, fontweight='bold', pad=15)

    nx.draw_networkx_nodes(
        G, pos_layout,
        node_color=colors, # type: ignore
        node_size=1200,
        edgecolors='#222222',
        linewidths=1.5,
        alpha=0.9
    )

    nx.draw_networkx_edges(
        G, pos_layout,
        edge_color='#555555',
        width=2.0,
        arrowstyle='-|>',
        arrowsize=20,
        node_size=1400
    )

    # Label edges with support weight ratios
    edge_labels = {(u, v): f"{d['weight']:.2f}" for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos_layout, edge_labels=edge_labels, font_size=9, font_color="#333333")

    # High-contrast label text overlay
    for node_name, pos in pos_layout.items():
        idx = list(G.nodes()).index(node_name)
        bg_color = colors[idx]

        luminance = 0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2]
        text_color = 'white' if luminance < 0.6 else 'black'

        plt.text(
            pos[0], pos[1], labels[node_name],
            color=text_color,
            fontsize=9,
            fontweight='bold',
            horizontalalignment='center',
            verticalalignment='center'
        )

    plt.axis('off')
    plt.tight_layout()

    if show:
        plt.show()

def generate_nice_colors(num_colors: int) -> list:
    """
        Generates a list of distinct, visually cohesive RGB colors using HSV.

        Args:
            num_colors: Number of unique colors needed.
        Returns:
            A list of [R, G, B] lists with values scaled between 0.0 and 1.0.
    """
    colors = []
    for i in range(num_colors):
        # Evenly space the hues across the color wheel, then add a tiny random jitter
        hue = (i / num_colors) + random.uniform(-0.02, 0.02)
        hue = hue % 1.0  # Keep it wrapped around 0-1

        # Keep saturation and brightness high and consistent for a clean look
        saturation = random.uniform(0.65, 0.80)  # Rich but not blinding
        value = random.uniform(0.75, 0.85)       # Bright and clean

        # Convert to RGB (colorsys outputs 0.0 - 1.0 floats)
        rgb = list(colorsys.hsv_to_rgb(hue, saturation, value))
        colors.append(rgb)

    # Shuffle so that adjacent blocks in your loop don't get near-identical gradients
    random.shuffle(colors)
    return colors

def find_feasible_block_sequence(support_graph: Dict[str, SupportNode]) -> List[str]:
    """
        Determines a feasible sequence of block placements based on the support graph.
        Uses a topological sort approach to ensure all dependencies are respected.

        Args:
            support_graph: Dict mapping block names to their SupportNode with support relationships.
        Returns:
            A list of block names in the order they should be placed.
    """
    G = nx.DiGraph()

    # Build directed graph from support relationships
    # Also check if all the support scores exceed the stability threshold.
    for name, node in support_graph.items():
        G.add_node(name)
        if not node.supporting_objects:
            print(f"Block '{name}' has no supports, invalid structure!")
            return []

        tentative_total_support_score = 0.0
        for parent_name, (score, _) in node.supporting_objects.items():
            tentative_total_support_score += score
            if parent_name in support_graph:
                G.add_edge(parent_name, name)

        if (tentative_total_support_score < node.support_threshold and len(node.supporting_objects) < 2) or \
            tentative_total_support_score < 1e-3: # If the total support score is negligible, treat it as unsupported
            print(f"Block '{name}' with support score {tentative_total_support_score:.2f} "
                  f"does not meet support threshold with current supports, invalid structure!")
            return []

    try:
        # Perform topological sort to find a valid placement sequence
        placement_sequence = list(nx.topological_sort(G))
        return placement_sequence
    except nx.NetworkXUnfeasible:
        raise ValueError("The support graph contains cycles, no valid placement sequence exists.")

def animate_construction_sequence(init_data: dict, goal_data: dict, placement_sequence: list, color_array: list, 
                                  interval: int = 800, show: bool = False, title: str = "Construction Animation") -> FuncAnimation:
    """
        Animates blocks moving from initial positions to target positions.
        - Unplaced blocks wait at their initial position in full color.
        - Future positions at the construction site are shown as faint gray references.
        - Placed blocks snap to their goal positions in full color.
        
        Args:
            init_data: Dict from init.yaml containing starting positions.
            goal_data: Dict from goal.yaml containing target positions.
            placement_sequence: List of block names in assembly order.
            color_array: List of RGB(A) colors corresponding 1:1 to the placement_sequence order.
            interval: Animation step delay in milliseconds.
            show: Whether to display the animation.
            title: Title for the animation.
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    # 1. Map blocks to their sequence color for easy lookup
    block_colors = {name: color_array[i] for i, name in enumerate(placement_sequence)}

    # Pre-calculate meshes for both states to avoid re-generating during frames
    init_meshes = {}
    goal_meshes = {}
    all_vertices = []

    for name in placement_sequence:
        if name not in init_data or name not in goal_data:
            continue

        # Initial position mesh
        init_mesh = make_box_mesh(init_data[name]['size'], init_data[name]['position'])
        init_meshes[name] = init_mesh
        all_vertices.append(init_mesh.vertices)

        # Target goal position mesh
        goal_mesh = make_box_mesh(goal_data[name]['size'], goal_data[name]['position'])
        goal_meshes[name] = goal_mesh
        all_vertices.append(goal_mesh.vertices)

    # Calculate global workspace dimensions dynamically to fit both areas
    flat_verts = np.vstack(all_vertices)
    max_x, min_x = flat_verts[:, 0].max(), flat_verts[:, 0].min()
    max_y, min_y = flat_verts[:, 1].max(), flat_verts[:, 1].min()
    max_z = flat_verts[:, 2].max()
    margin = 2.0

    def update(frame_idx):
        ax.clear()

        ax.set_title(f"Assembly Sequence Map: Step {frame_idx}/{len(placement_sequence)}", 
                     fontsize=14, fontweight='bold', pad=20)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
        ax.set_zlim(0, max_z + margin)
        ax.view_init(elev=25, azim=-45)

        # Ground Grid Floor
        extent_x = max(abs(min_x), abs(max_x)) + margin
        extent_y = max(abs(min_y), abs(max_y)) + margin
        gx, gy = np.meshgrid(np.linspace(-extent_x, extent_x, 12), np.linspace(-extent_y, extent_y, 12))
        ax.plot_wireframe(gx, gy, np.zeros_like(gx), color=(0.7, 0.7, 0.7, 0.12), linewidth=0.8)

        # Separate what has been moved vs what is still waiting
        built_so_far = set(placement_sequence[:frame_idx])

        for name in placement_sequence:
            if name not in init_meshes:
                continue

            color = block_colors[name]
            face_color = color[:3]
            alpha_val = color[3] if len(color) == 4 else 0.8

            # SCENARIO A: Block has been built / moved to goal
            if name in built_so_far:
                # Draw at GOAL position in full color
                tris = goal_meshes[name].vertices[goal_meshes[name].faces]
                poly = Poly3DCollection(tris, alpha=alpha_val)
                poly.set_facecolor(face_color)
                poly.set_edgecolor([0.1, 0.1, 0.1])
                poly.set_linewidth(0.8)
                ax.add_collection3d(poly)

            # SCENARIO B: Block is still waiting at the staging area
            else:
                # 1. Draw at INIT position in full color
                tris_init = init_meshes[name].vertices[init_meshes[name].faces]
                poly_init = Poly3DCollection(tris_init, alpha=alpha_val)
                poly_init.set_facecolor(face_color)
                poly_init.set_edgecolor([0.2, 0.2, 0.2])
                poly_init.set_linewidth(0.8)
                ax.add_collection3d(poly_init)

                # 2. Draw phantom reference at GOAL position (faint grey wireframe)
                tris_goal = goal_meshes[name].vertices[goal_meshes[name].faces]
                poly_phantom = Poly3DCollection(tris_goal, alpha=0.04)
                poly_phantom.set_facecolor([0.85, 0.85, 0.85])
                poly_phantom.set_edgecolor([0.6, 0.6, 0.6, 0.2])
                poly_phantom.set_linewidth(0.5)
                ax.add_collection3d(poly_phantom)
            plt.savefig(f"movies/{title}_animation_step_{frame_idx}.pdf", bbox_inches='tight')

    total_frames = len(placement_sequence) + 1
    anim = FuncAnimation(fig, update, frames=total_frames, interval=interval, repeat=True) # type: ignore

    plt.tight_layout()
    if show:
        plt.show()
    return anim

def save_construction_sequence_frames(init_data: dict, goal_data: dict, placement_sequence: list, color_array: list, 
                                      file_format: str = "pdf", title: str = "Construction") -> None:
    """
        Generates and saves static frame files for each step of the assembly sequence.
        
        Args:
            init_data: Dict from init.yaml containing starting positions.
            goal_data: Dict from goal.yaml containing target positions.
            placement_sequence: List of block names in assembly order.
            color_array: List of RGB(A) colors corresponding 1:1 to the placement_sequence order.
            file_format: "pdf" (recommended for LaTeX vectors) or "png" (faster/smaller).
            title: Title prefix for the saved files.
    """
    # 1. Ensure the output directory exists
    output_dir = f"movies/{title}"
    os.makedirs(output_dir, exist_ok=True)

    # 2. Map blocks to their sequence color for easy lookup
    block_colors = {name: color_array[i] for i, name in enumerate(placement_sequence)}

    # Pre-calculate meshes to avoid re-generating during the loop
    init_meshes = {}
    goal_meshes = {}
    all_vertices = []

    for name in placement_sequence:
        if name not in init_data or name not in goal_data:
            continue
        init_mesh = make_box_mesh(init_data[name]['size'], init_data[name]['position'])
        init_meshes[name] = init_mesh
        all_vertices.append(init_mesh.vertices)

        goal_mesh = make_box_mesh(goal_data[name]['size'], goal_data[name]['position'])
        goal_meshes[name] = goal_mesh
        all_vertices.append(goal_mesh.vertices)

    flat_verts = np.vstack(all_vertices)
    max_x, min_x = flat_verts[:, 0].max(), flat_verts[:, 0].min()
    max_y, min_y = flat_verts[:, 1].max(), flat_verts[:, 1].min()
    max_z = flat_verts[:, 2].max()
    margin = 2.0

    # Calculate global grid extent
    extent_x = max(abs(min_x), abs(max_x)) + margin
    extent_y = max(abs(min_y), abs(max_y)) + margin
    gx, gy = np.meshgrid(np.linspace(-extent_x, extent_x, 12), np.linspace(-extent_y, extent_y, 12))

    # Total frames: from step 0 (nothing built) to len(sequence) (fully built)
    total_steps = len(placement_sequence) + 1

    print(f"Generating {total_steps} individual frames...")

    # 3. Loop through each step explicitly
    for frame_idx in range(total_steps):
        # Create a fresh figure instance for every single frame to prevent memory accumulation
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')

        ax.set_title(f"Assembly Sequence Map: Step {frame_idx}/{len(placement_sequence)}", 
                     fontsize=14, fontweight='bold', pad=20)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
        ax.set_zlim(0, max_z + margin)
        ax.view_init(elev=25, azim=-45)

        # Ground Grid Floor
        ax.plot_wireframe(gx, gy, np.zeros_like(gx), color=(0.7, 0.7, 0.7, 0.12), linewidth=0.8)

        # Determine structural status at this specific step index
        built_so_far = set(placement_sequence[:frame_idx])

        for name in placement_sequence:
            if name not in init_meshes:
                continue

            color = block_colors[name]
            face_color = color[:3]
            alpha_val = color[3] if len(color) == 4 else 0.8

            if name in built_so_far:
                # SCENARIO A: Block placed at GOAL position
                tris = goal_meshes[name].vertices[goal_meshes[name].faces]
                poly = Poly3DCollection(tris, alpha=alpha_val)
                poly.set_facecolor(face_color)
                poly.set_edgecolor([0.1, 0.1, 0.1])
                poly.set_linewidth(0.8)
                ax.add_collection3d(poly)
            else:
                # SCENARIO B: Block still waiting at STAGING area
                tris_init = init_meshes[name].vertices[init_meshes[name].faces]
                poly_init = Poly3DCollection(tris_init, alpha=alpha_val)
                poly_init.set_facecolor(face_color)
                poly_init.set_edgecolor([0.2, 0.2, 0.2])
                poly_init.set_linewidth(0.8)
                ax.add_collection3d(poly_init)

                # Blueprint reference outline at target
                tris_goal = goal_meshes[name].vertices[goal_meshes[name].faces]
                poly_phantom = Poly3DCollection(tris_goal, alpha=0.04)
                poly_phantom.set_facecolor([0.85, 0.85, 0.85])
                poly_phantom.set_edgecolor([0.6, 0.6, 0.6, 0.2])
                poly_phantom.set_linewidth(0.5)
                ax.add_collection3d(poly_phantom)

        # 4. Save the single canvas snapshot cleanly
        filename = f"{output_dir}/{title}_step_{frame_idx:02d}.{file_format}"
        plt.tight_layout()
        plt.savefig(filename, bbox_inches='tight', dpi=300)

        # 5. Crucial: close the figure handle completely to prevent massive RAM leaks
        plt.close(fig)

    print(f"All frames successfully saved to the '{output_dir}/' directory.")