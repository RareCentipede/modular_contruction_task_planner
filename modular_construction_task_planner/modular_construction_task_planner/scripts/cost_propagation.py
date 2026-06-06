import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from typing import Dict, List, Tuple, cast
from scipy.spatial import cKDTree

from modular_construction_task_planner.eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object, PosEntity, ShadowBox
from modular_construction_task_planner.scripts.ordered_landmarks_planner import compute_dists_from_points_to_vector

OBJ_WIDTH = 3.0
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

def perform_cost_propagation(world: World, shadow_boxes: Dict[str, ShadowBox], verbose: bool = False) -> None:
    available_entities, all_entities, entity_positions = extract_available_positions_from_world(world, shadow_boxes)
    entity_positions = np.array(entity_positions)
    shadow_boxes_names = [sb.name for sb in shadow_boxes.values()]

    for obj in available_entities:
        obj = cast(Object, obj)
        obj_pos = np.array(world.pose_dict[obj.at.value].position) # type: ignore
        goal_pos = np.array(world.pose_dict[obj.goal.value].position) # type: ignore
        obj_to_goal_vec = goal_pos - obj_pos
        dists, scalings = compute_dists_from_points_to_vector(entity_positions, obj_to_goal_vec, obj_pos)

        dists[(scalings < 0) | (scalings > 1)] = -1 # Set a false value to filter out irrelevant entities in the next step

        for i, dist in enumerate(dists):
            if dist < 0:
                continue  # Skip irrelevant entities

            if dist < (OBJ_WIDTH/2 + ROBOT_WIDTH):
                nusance = list(all_entities.values())[i]
                if nusance.name == obj.name or nusance.name == obj.name + "_shadow_box":
                    continue  # Skip self and own shadow box

                if verbose:
                    print(f"Object '{obj.name}' has a nusance '{nusance.name}' at index {i} with distance {dist:.2f} to its path.")
                if nusance.name in shadow_boxes_names:
                    shadow_nusnace = all_entities[nusance.name]
                    shadow_nusnace = cast(ShadowBox, shadow_nusnace)
                    host_obj_name = shadow_nusnace.host.value
                    host_obj = world.entities.get_entities(host_obj_name) # type: ignore
                    host_obj = cast(Object, host_obj)
                    host_obj.propagated_cost += 1 - (dist / (np.sqrt(2)*OBJ_WIDTH/2 + ROBOT_WIDTH))
                else:
                    host_obj_name = nusance.name
                    host_obj = world.entities.get_entities(host_obj_name) # type: ignore
                    host_obj = cast(Object, host_obj)
                    host_obj.propagated_cost -= 1 - (dist / (np.sqrt(2)*OBJ_WIDTH/2 + ROBOT_WIDTH))

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

def visualize_cost_propagation(world: World) -> None:
    obj_entities = world.entities.get_entities(Object)
    obj_entities = cast(List[Object], obj_entities)
    entity_positions = np.array([p.position[:2] for p in world.pose_dict.values()])

    # Choose a nice qualitative or sequential colormap
    cmap = plt.get_cmap('turbo') # type: ignore

    # Pick 5 random positions along the colormap (from 0.0 to 1.0)
    num_colors = len(obj_entities)
    random_indices = np.random.rand(num_colors)
    random_colors = cmap(random_indices)
    plt.figure(figsize=(10, 10))

    for i, entity in enumerate(obj_entities):
        entity = cast(Object, entity)
        pos_val = entity.at.value
        goal_pos_val = entity.goal.value
        if not pos_val or not goal_pos_val:
            continue

        pos = world.pose_dict[pos_val].position
        goal_pos = world.pose_dict[goal_pos_val].position
        cost = entity.propagated_cost
        color = random_colors[i]
        collision_threshold = np.sqrt(2)*OBJ_WIDTH/2 + ROBOT_WIDTH
        collision_region = patches.Circle((pos[0], pos[1]), collision_threshold, color=color, alpha=0.2, linewidth=2)
        collision_region_shadow = patches.Circle((goal_pos[0], goal_pos[1]), collision_threshold, color=color, alpha=0.2,
                                                 linestyle='--', linewidth=2)

        host = patches.Rectangle((pos[0] - OBJ_WIDTH/2, pos[1] - OBJ_WIDTH/2), OBJ_WIDTH, OBJ_WIDTH, color=color,
                                 alpha=0.8, linewidth=2)
        shadow = patches.Rectangle((goal_pos[0] - OBJ_WIDTH/2, goal_pos[1] - OBJ_WIDTH/2), OBJ_WIDTH, OBJ_WIDTH,
                                   alpha=0.5, linestyle='--', linewidth=2, facecolor='none', edgecolor=color)

        plt.gca().add_patch(collision_region)
        plt.gca().add_patch(collision_region_shadow)
        plt.gca().add_patch(host)
        plt.gca().add_patch(shadow)
        plt.arrow(pos[0], pos[1], goal_pos[0] - pos[0], goal_pos[1] - pos[1], linestyle='--', color=color, alpha=0.7,
                  head_width=0.05, linewidth=2)
        plt.text(pos[0], pos[1] + OBJ_WIDTH/2, f"{entity.name}\nCost: {cost:.2f}", fontsize=9, ha='right', va='bottom')
        # plt.scatter(pos[0], pos[1], color=color, s=100, label=f"{entity.name} (Cost: {cost:.2f})")
        # plt.scatter(goal_pos[0], goal_pos[1], color=color, alpha=0.5, s=100, marker='X') # type: ignore
        # plt.plot([pos[0], goal_pos[0]], [pos[1], goal_pos[1]], linestyle='--')

    x_max = max(entity_positions[:, 0]) + 1
    y_max = max(entity_positions[:, 1]) + 1
    x_min = min(entity_positions[:, 0]) - 1
    y_min = min(entity_positions[:, 1]) - 1
    l = max(x_max, abs(x_min), y_max, abs(y_min))
    plt.xlim(-l, l)
    plt.ylim(-l, l)

    plt.title("Cost Propagation Visualization")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid()
    plt.show()