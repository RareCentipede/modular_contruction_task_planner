import trimesh
import colorsys
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh import Trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
from typing import Dict, List, Tuple, cast, Optional
from dataclasses import dataclass, field

from modular_construction_task_planner.eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object

@dataclass
class SupportNode:
    name: str
    area: float
    support_threshold: float
    total_support_ratio: float = 0.0
    current_support_score: float = 0.0
    supporting_objects: List[Tuple[str, float, bool]] = field(default_factory=list) # Objects that support THIS object.
    # (name, support_score, is_placed)
    supported_objects: List[Tuple[str, float, bool]] = field(default_factory=list) # Objects that this object SUPPORTS
    support_combo_dict: Dict[Tuple[str, ...], float] = field(default_factory=dict) # For future use: mapping of specific combinations of supporting objects to their combined support score.

    @property
    def supported(self) -> bool:
        return self.current_support_score >= self.support_threshold

# Assume cubes
def create_support_relation_graph(world: World, support_ratio_threshold: float = 0.7) -> Tuple[Trimesh, Dict[str, SupportNode]]:
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

        support_data, overall_support_area_ratio = compute_placement_stability(obj, candidate_objs, [], ground_mesh) # type: ignore
        supporting_objs = [(name, score, False) for name, score in support_data.items() if name != 'area' and name != 'g']
        support_node = SupportNode(
            name=obj.name,
            area=support_data['area'],
            total_support_ratio=overall_support_area_ratio,
            support_threshold=support_ratio_threshold,
            supporting_objects=supporting_objs
        )
        support_graph[obj.name] = support_node

        if 'g' in support_data:
            support_node.supporting_objects.append(('g', support_data['g'], False))

    for support_node in support_graph.values():
        for name, score, _ in support_node.supporting_objects:
            if name in support_graph:
                support_graph[name].supported_objects.append((support_node.name, score, False))

    return ground_mesh, support_graph

def compute_placement_stability(object: Object | Trimesh,
                                candidate_support_objs: List[Object] | List[Trimesh],
                                supp_names: List[str],
                                ground_plane: Trimesh) -> Tuple[Dict[str, float], float]:
    """
        Compute the stability of placing the object at its goal position based on support area analysis.
    """
    support_data = {}
    support_polys = []
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

        support_polygon = contact_polygon.convex_hull
        support_score = support_polygon.area / footprint.area
        support_data[supp_name] = support_score
        support_polys.append(support_polygon)

    # 3. Aggregate total support
    if support_polys:
        # Combine all supporting polygons using shapely union to handle overlaps safely
        combined_support = unary_union(support_polys).convex_hull
        # Intersect with object footprint so we don't count "phantom support" outside the block
        actual_support = combined_support.intersection(footprint)
        overall_support_area_ratio = actual_support.area / footprint.area

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

def visualize_goal_structure(goal_data: dict, title: str = "Target Goal Structure Layout", show: bool = True) -> List[List[float]]:
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
    unique_colors = generate_nice_colors(len(goal_data))

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
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
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

    ax.legend(
        handles=legends,
        loc='upper left',
        bbox_to_anchor=(1.05, 1), # Moves the legend just to the right of the plot area
        borderaxespad=0.,
        title="Objects List",
        title_fontproperties={'weight': 'bold'}
    )

    plt.tight_layout()
    if show:
        plt.show()

    return unique_colors

def visualize_support_node_graph(support_graph: Dict[str, 'SupportNode'],
                                 goal_data: Optional[Dict] = None,
                                 colors: List[List[float]] = [],
                                 title: str = "Structural Support & Stability Graph",
                                 show: bool = True):
    """
        Visualizes the support relations using a dictionary of SupportNode objects.
        
        Args:
            support_graph: Dict[str, SupportNode] mapping block names to their SupportNode.
            goal_data: Optional dictionary from goal.yaml containing 'color' properties. 
                    If omitted, nodes are color-coded by their stability status.
            title: Title of the generated graph plot.
    """
    G = nx.DiGraph()
    node_colors = colors
    labels = {}

    # 1. Build nodes and determine coloring strategy
    for name, node in support_graph.items():
        G.add_node(name)

        # Build text label to show at the center of the node
        # Displays name, current support score, and required threshold
        labels[name] = f"{name}\n({node.current_support_score:.2f}/{node.support_threshold:.2f})"

        # Color mapping logic
        if not node_colors:
            if node.supported:
                node_colors.append([0.2, 0.65, 0.3]) # Stable Green
            else:
                node_colors.append([0.85, 0.3, 0.2]) # Unstable Red

    # 2. Extract directed edges
    # Standardizing direction: 'supporting_objects' means Parent -> Child (This object)
    for name, node in support_graph.items():
        for parent_name, score, is_placed in node.supporting_objects:
            # Add edge from the block that provides support to the block receiving it
            if parent_name in support_graph:
                G.add_edge(parent_name, name, weight=score)

    # 3. Graph Presentation and Layout
    plt.figure(figsize=(14, 10))
    plt.title(title, fontsize=14, fontweight='bold', pad=15)

    try:
        # Hierarchical layout prioritizing bottom-up structure mapping
        pos_layout = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot', args='-Grankdir=BT')
    except (ImportError, OSError):
        # Reliable spring/force-directed layout fallback
        pos_layout = nx.spring_layout(G, k=1.5, seed=42)

    # 4. Render Graph Elements
    nx.draw_networkx_nodes(
        G, pos_layout, 
        node_color=node_colors, 
        node_size=2800, 
        edgecolors='#222222', 
        linewidths=1.5,
        alpha=0.9
    )

    nx.draw_networkx_edges(
        G, pos_layout, 
        edge_color='#555555', 
        width=2.0, 
        arrowstyle='-|>', 
        arrowsize=22, 
        node_size=2800
    )

    # Extract weights from the graph edges to label them
    edge_labels = {(u, v): f"{d['weight']:.2f}" for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos_layout, edge_labels=edge_labels, font_size=8, font_color="#333333")

    # Determine highly readable text contrast dynamically based on node color background brightness
    for node_name, pos in pos_layout.items():
        idx = list(G.nodes()).index(node_name)
        bg_color = node_colors[idx]

        # Compute simple luminance to pick text color
        luminance = 0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2]
        text_color = 'white' if luminance < 0.6 else 'black'

        plt.text(
            pos[0], pos[1], labels[node_name],
            color=text_color,
            fontsize=8,
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
        for parent_name, score, _ in node.supporting_objects:
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