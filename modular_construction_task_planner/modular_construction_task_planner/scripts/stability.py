import numpy as np

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
    obj_with_goals = [obj for obj in objs if obj.goal.value]
    for obj in obj_with_goals:
        goal_pose = world.pose_dict.get(obj.goal.value) # type: ignore
        goal_pose_dict[obj.name] = goal_pose
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
        candidate_position_idx = goal_positions[goal_positions[:, 2] < goal_pos[2]] # Objects below the current object
        candidate_objs = obj_with_goals[candidate_position_idx]

        support_data, overall_support_area_ratio = compute_placement_stability(obj, candidate_objs, ground_mesh) # type: ignore
        supporting_objs = [(name, score, False) for name, score in support_data.items() if name != 'area' and name != 'g']
        support_node = SupportNode(
            name=obj.name,
            area=support_data['area'],
            total_support_ratio=overall_support_area_ratio,
            support_threshold=support_ratio_threshold,
            supporting_objects=supporting_objs
        )
        support_graph[obj.name] = support_node

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