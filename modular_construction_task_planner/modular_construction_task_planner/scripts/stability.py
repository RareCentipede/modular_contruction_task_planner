import trimesh
import numpy as np
import matplotlib.pyplot as plt

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

    @property
    def supported(self) -> bool:
        return self.current_support_score >= self.support_threshold

# Assume cubes
def create_support_relation_graph(world: World, support_ratio_threshold: float = 0.7) -> Dict[str, SupportNode]:
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

    return support_graph

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

def visualize_goal_structure(goal_data: dict, title: str = "Target Goal Structure Layout"):
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
    legend_patches = []

    # Iterate and draw each block
    for name, data in goal_data.items():
        if name == 'robot' or 'position' not in data:
            continue
            
        pos = data['position']
        size = data['size']
        # Default to a nice slate blue if color isn't provided in the goal dict
        color = data.get('color', [0.2, 0.5, 0.8]) 
        
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
    
    plt.tight_layout()
    plt.show()