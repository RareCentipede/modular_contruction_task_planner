import numpy as np

from trimesh import Trimesh
from shapely.geometry import Polygon
from typing import Dict, List, Tuple, cast, Optional
from dataclasses import dataclass, field

from modular_construction_task_planner.eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object

@dataclass
class SupportNode:
    name: str
    area: float
    support_threshold: float
    total_support_area: float = 0.0
    current_support_score: float = 0.0
    supporting_objects: List[Tuple[str, bool]] = field(default_factory=list) # Objects that support THIS object.
    # True if the supporting object is placed, False otherwise.
    supported_objects: List[str] = field(default_factory=list) # Objects that this object SUPPORTS

    @property
    def supported(self) -> bool:
        return self.current_support_score >= self.support_threshold

# Assume cubes
def create_support_relation_graph(world: World) -> Dict[str, SupportNode]:
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

    return support_graph

def compute_placement_stability(object: Object,
                                candidate_support_objs: List[Object],
                                ground_plane: Trimesh,
                                support_ratio_threshold: float = 0.7) -> Tuple[Dict[str, float], float]:
    """
        Compute the stability of placing the object at its goal position based on support area analysis.
    """
    support_data = {}
    support_polys = []
    support_vertices = []
    overall_support_area_ratio = 0.0

    footprint = Polygon(object.mesh.vertices[:, :2]).convex_hull
    ground_contact_polygon = get_contact_polygon(object.mesh, ground_plane)

    if ground_contact_polygon:
        support_data['g'] = 1.0 # Ground provides full support
        overall_support_area_ratio = ground_contact_polygon.area / footprint.area
        return support_data, overall_support_area_ratio

    for supp in candidate_support_objs:
        contact_polygon = get_contact_polygon(object.mesh, supp.mesh)
        if not contact_polygon:
            continue

        support_polygon = contact_polygon.convex_hull
        support_score = support_polygon.area / footprint.area
        support_data[supp.name] = support_score
        support_polys.append(support_polygon)

    if support_polys:
        support_vertices = np.vstack([np.array(poly.exterior.coords) for poly in support_polys])
        overall_support_polygon = Polygon(support_vertices).convex_hull
        overall_support_area_ratio = overall_support_polygon.area / footprint.area

    return support_data, overall_support_area_ratio

def get_contact_polygon(mesh_above: Trimesh, mesh_below: Trimesh,
                        contact_z_tolerance: float = 0.02) -> Optional[Polygon]:
    """
        Given two meshes already transformed to world frame,
        find their contact polygon in the XY plane.
    """
    # Find the contact Z — bottom of upper mesh
    contact_z = mesh_above.bounds[0][2] - contact_z_tolerance  # min Z of upper mesh

    # Slice both meshes at contact Z
    slice_above = mesh_above.section(
        plane_origin=[0, 0, contact_z],
        plane_normal=[0, 0, 1]
    )
    slice_below = mesh_below.section(
        plane_origin=[0, 0, contact_z],
        plane_normal=[0, 0, 1]
    )

    if slice_above is None or slice_below is None:
        return None

    # Convert cross-sections to 2D polygons
    poly_above, _ = slice_above.to_planar()
    poly_below, _ = slice_below.to_planar()

    # Shapely intersection
    contact = poly_above.polygons_full[0].intersection(
                poly_below.polygons_full[0])

    return contact if not contact.is_empty else None