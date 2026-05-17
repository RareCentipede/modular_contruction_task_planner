import numpy as np

from scipy.spatial import cKDTree, ConvexHull
from typing import Dict, List, Tuple
from dataclasses import dataclass, field
from enum import Enum

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
    # For each object in the world, determine which other objects it is supporting and which objects are supporting it
    # Calculate the contact area between objects to determine the strength of support relationships
    # Store this information in a graph structure (e.g., adjacency list or matrix) for later analysis

    return support_graph

def compute_placement_stability(world: World,
                                goal_pos_dict: Dict[str, List[float]],
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