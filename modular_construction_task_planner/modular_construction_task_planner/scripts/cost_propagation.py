import numpy as np
import matplotlib.pyplot as plt

from typing import Dict, List, Tuple, cast
from scipy.spatial import cKDTree

from eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object, PosEntity, ShadowBox
from modular_construction_task_planner.scripts.ordered_landmarks_planner import compute_dists_from_points_to_vector

OBJ_WIDTH = 0.15
ROBOT_WIDTH = 0.3

def spawn_shadow_boxes(world: World) -> Dict[str, ShadowBox]:
    """
        Shadow boxes: [host_entity_name] -> ShadowBox: Object
    """

    shadow_boxes = {}
    obj_entities = world.entities.get_entities(Object)

    if not obj_entities:
        raise ValueError("No object entities found in the world to spawn shadow boxes.")

    obj_entities = cast(List[Object], obj_entities)
    for obj in obj_entities:
        if not obj.goal:
            continue  # Skip objects without a goal position

        obj_name = obj.name
        shadow_box_name = f"{obj_name}_shadow_box"
        shadow_box = ShadowBox(shadow_box_name)
        shadow_box.host.value = obj_name
        shadow_box.at.value = obj.goal.value
        shadow_boxes[obj.name] = shadow_box

    return shadow_boxes

def perform_cost_propagation(world: World, shadow_boxes: Dict[str, ShadowBox]) -> None:
    available_entities, all_entities, entity_positions = extract_available_positions_from_world(world, shadow_boxes)
    entity_positions = np.array(entity_positions)
    shadow_boxes_names = list(shadow_boxes.keys())

    for obj in available_entities:
        obj = cast(Object, obj)
        obj_pos = np.array(world.pose_dict[obj.at.value].position) # type: ignore
        goal_pos = np.array(world.pose_dict[obj.goal.value].position) # type: ignore
        obj_to_goal_vec = goal_pos - obj_pos
        dists, scalings = compute_dists_from_points_to_vector(entity_positions, obj_to_goal_vec, obj_pos)

        dists[(scalings >= 0) & (scalings <= 1)] = -1 # Set a false value to filter out irrelevant entities in the next step

        for i, dist in enumerate(dists):
            if dist < 0: 
                continue  # Skip irrelevant entities

            if dist < (OBJ_WIDTH + ROBOT_WIDTH):
                nusance = list(all_entities.values())[i]
                if nusance.name == obj.name:
                    continue  # Skip self

                if nusance.name in shadow_boxes_names:
                    host_obj_name = shadow_boxes[nusance.name].host.value
                    host_obj = world.entities.get_entities(host_obj_name) # type: ignore
                    host_obj = cast(Object, host_obj)
                    host_obj.propagated_cost += dist
                else:
                    host_obj_name = nusance.name
                    host_obj = world.entities.get_entities(host_obj_name) # type: ignore
                    host_obj = cast(Object, host_obj)
                    host_obj.propagated_cost -= dist

def extract_available_positions_from_world(world: World, shadow_boxes: Dict[str, ShadowBox]) -> \
    Tuple[List[Object], Dict[str, Object], List[Tuple[float, float]]]:
    available_entities = []
    entity_positions = []
    all_entities = {}

    for box_host_name, shadow_box in shadow_boxes.items():
        goal_pos = shadow_box.at.value
        if not goal_pos:
            continue

        host_entity = world.entities.get_entities(box_host_name)
        host_entity = cast(Object, host_entity)
        if host_entity.at.value == goal_pos or not host_entity.at.value:
            continue  # Skip if the object is already at its goal position

        pos_val = host_entity.at.value
        pos = world.pose_dict[pos_val].position
        shadow_pos = world.pose_dict[goal_pos].position

        available_entities.append(host_entity)
        entity_positions.append(pos)
        entity_positions.append(shadow_pos)
        all_entities[host_entity.name] = host_entity
        all_entities[shadow_box.name] = shadow_box

    return available_entities, all_entities, entity_positions