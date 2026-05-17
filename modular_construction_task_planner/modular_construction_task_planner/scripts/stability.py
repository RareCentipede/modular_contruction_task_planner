import numpy as np

from scipy.spatial import cKDTree, ConvexHull
from typing import Dict, List, Tuple, cast
from dataclasses import dataclass, field
from enum import Enum

from modular_construction_task_planner.eas.core import Pose, World
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
    goal_pos_dict = {}
    # For each object in the world, determine which other objects it is supporting and which objects are supporting it
    # Calculate the contact area between objects to determine the strength of support relationships
    # Store this information in a graph structure (e.g., adjacency list or matrix) for later analysis

    objs = world.entities.get_entities(Object)
    objs = cast(List[Object], objs)
    obj_with_goals = [obj for obj in objs if obj.goal.value]
    for obj in obj_with_goals:
        goal_pos = world.pose_dict.get(obj.goal.value) #type: ignore
        goal_pos_dict[obj.name] = goal_pos

    goal_poses = np.array([goal_pose.position for goal_pose in goal_pos_dict.values()])
    goal_positions = np.array([[g[0], g[1], g[2]] for g in goal_poses])
    world_tree = cKDTree(goal_positions)

    for obj in obj_with_goals:
        support_data, overall_support_area_ratio = compute_placement_stability(world, goal_pos_dict, world_tree, obj)

    return support_graph

def compute_placement_stability(world: World,
                                goal_pos_dict: Dict[str, Pose],
                                world_tree: cKDTree, 
                                object: Object,
                                support_ratio_threshold: float = 0.7) -> Tuple[Dict[str, float], float]:
    """
        Compute the stability of placing the object at its goal position based on support area analysis.
    """
    support_data = {}
    overall_support_area_ratio = 0.0
    # Query objects whose goal position is below (lower z position value) the goal position of the current object
    # Determine which objects the current object would be in contact with at the goal

    # Project them onto the ground plane and compute the support area for each supporting object
    # Get convex hull of the support area, compute the ratio between the support area and the total area of the supported
    # object.
    # If the overall support ratio is below the stability threshold, just return empty dictionary and the score.
    # Structure is not stable.

    # Each supporting object's support score is its own support area over the total support area.
    # Return a support_data dictionary and the total support are ratio.
    # Keys are supporintg object names, values are support scores of each object

    return support_data, overall_support_area_ratio