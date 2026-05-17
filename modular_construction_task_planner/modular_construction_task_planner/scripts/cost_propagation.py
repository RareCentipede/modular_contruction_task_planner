import numpy as np
import matplotlib.pyplot as plt

from typing import Dict, List, Tuple, cast

from eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object, PosEntity, ShadowBox
from modular_construction_task_planner.scripts.ordered_landmarks_planner import compute_dists_from_points_to_vector

def spawn_shadow_boxes(world: World) -> Dict[str, ShadowBox]:
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
    available_entities, entity_positions = extract_available_positions_from_world(world, shadow_boxes)

    for obj in available_entities:
        obj = cast(Object, obj)

def extract_available_positions_from_world(world: World, shadow_boxes: Dict[str, ShadowBox]) -> \
    Tuple[List[Object], List[Tuple[float, float]]]:
    available_entities = []
    entity_positions = []
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

        available_entities.append(host_entity)
        entity_positions.append(pos)

    return available_entities, entity_positions