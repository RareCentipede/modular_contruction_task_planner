import numpy as np

from yaml import safe_load
from typing import Dict, List, Tuple, cast

from eas.core import (
    load_domains,
    Entities,
    World,
    Pose,
    State
)
from modular_construction_task_planner.block_domain import (
    Object,
    PosEntity,
    Robot
)

def parse_configs_to_world(config_name: str | Dict, problem_config_path: str | Dict) -> World:
    if isinstance(config_name, str) and isinstance(problem_config_path, str):
        init_path = problem_config_path + config_name + "/init.yaml"
        goal_path = problem_config_path + config_name + "/goal.yaml"

        with open(init_path, 'r') as f:
            init_config = safe_load(f)
            f.close()

        with open(goal_path, 'r') as f:
            goal_config = safe_load(f)
            f.close()
    elif isinstance(config_name, Dict) and isinstance(problem_config_path, Dict):
        init_config = config_name
        goal_config = problem_config_path

    domains = create_domains(init_config, goal_config)
    load_domains(domains)
    entities = create_entities(domains)
    pose_dict = assign_entities_variable_values_and_create_pose_dict(init_config, goal_config, entities)
    goal_state = define_goal_state(entities)
    world = World(entities, pose_dict=pose_dict, goal_state=goal_state)

    return world

def load_configs_to_dict(config_name: str, problem_config_path: str) -> Tuple[Dict, Dict]:
    init_path = problem_config_path + config_name + "/init.yaml"
    goal_path = problem_config_path + config_name + "/goal.yaml"

    with open(init_path, 'r') as f:
        init_config = safe_load(f)
        f.close()

    with open(goal_path, 'r') as f:
        goal_config = safe_load(f)
        f.close()

    return init_config, goal_config

def create_domains(init_config: Dict, goal_config: Dict) -> Dict[str, Tuple]:
    idx = 1
    block_var_domain = []
    pos_var_domain = []
    robo_pos_var_domain = []
    bool_var_domain = (True, False)
    sides = ['front', 'back', 'left', 'right']

    for obj_name in init_config.keys():
        pos_name = 'p' + str(idx)
        pos_var_domain.append(pos_name)

        if obj_name != "robot":
            for side in sides:
                pick_target_name = f'{obj_name}_pick_target_{side}'
                robo_pos_var_domain.append(pick_target_name)
            block_var_domain.append(obj_name)
        else:
            robot_pos_name = 'r_init_pos'
            robo_pos_var_domain.append(robot_pos_name)

        idx += 1

    for obj_name in goal_config.keys():
        pos_name = 'p' + str(idx)
        pos_var_domain.append(pos_name)

        for side in sides:
            place_target_name = f'{obj_name}_place_target_{side}'
            robo_pos_var_domain.append(place_target_name)
        idx += 1

    pos_var_domain.append('g')
    pos_var_domain.append('')
    block_var_domain.append('g')
    block_var_domain.append('')
    domains = {
        'pos': tuple(pos_var_domain),
        'robo_pos': tuple(robo_pos_var_domain),
        'block': tuple(block_var_domain),
        'bool': bool_var_domain
    }

    return domains

def create_entities(domains: Dict[str, Tuple]) -> Entities:
    entities = []
    for block in domains['block']:
        entities.append(Object(block))
    for pos in domains['pos']:
        entities.append(PosEntity(pos))
    for robo_pos in domains['robo_pos']:
        entities.append(PosEntity(robo_pos))
    entities.append(Robot('robot'))

    return Entities(entities)

def assign_entities_variable_values_and_create_pose_dict(init_config: Dict, goal_config: Dict, entities: Entities) -> Dict[str, Pose]:
    pos_counter = 0
    init_pos_vals = []
    pose_dict = {}

    obj_entities = cast(List[Object], entities.get_entities(Object))
    pos_entities = cast(List[PosEntity], entities.get_entities(PosEntity))
    gnd_pos_entity = cast(PosEntity, entities.get_entities('g'))
    gnd_obj_entity = cast(Object, entities.get_entities('g'))

    # Parameterized distance: offset clear of the cube's geometric center (e.g., 1.5 meters)
    approach_distance = 0.5

    # --- Processing INITIAL CONFIGURATION (Pick Targets) ---
    for info, obj_entity, pos_entity in zip(init_config.values(), obj_entities[:-2], pos_entities[:-2]):
        pose = Pose(info['position'], info['orientation'])
        pose_dict[pos_entity.name] = pose
        obj_entity.at.value = pos_entity.name
        obj_entity.on.value = gnd_obj_entity.name
        obj_entity.dim = info['size']
    
        pos_entity.occupied_by.value = obj_entity.name
        pos_entity.on.value = gnd_pos_entity.name
        pos_entity.clear.value = False

        # Calculate base reachability targets and inward orientations on all four sides
        side_data = compute_side_positions_and_orientations(info['position'], info['orientation'], approach_distance)

        pick_target_labels = []
        for side_name, data in side_data.items():
            target_key = f'{obj_entity.name}_pick_target_{side_name}'

            # Use the calculated inward-facing orientation instead of the original block orientation
            pose_dict[target_key] = Pose(data['position'], data['orientation'])
            pick_target_labels.append(target_key)

        obj_entity.reachable_from = pick_target_labels

        init_pos_vals.append(info['position'])
        pos_counter += 1

    # --- Processing ROBOT INITIAL STATE ---
    robot_entity = cast(Robot, entities.get_entities('robot'))
    robot_entity.at.value = 'r_init_pos'
    pose_dict[robot_entity.at.value] = Pose(init_config['robot']['position'],
                                            init_config['robot']['orientation'])

    # --- Processing GOAL CONFIGURATION (Place Targets) ---
    for obj_name, info, pos_entity in zip(goal_config.keys(), goal_config.values(), pos_entities[pos_counter+1:-2]):
        pos_val = info['position']

        if pos_val not in init_pos_vals:
            pose = Pose(info['position'], info['orientation'])
            pose_dict[pos_entity.name] = pose
            pos_entity.clear.value = True
            pos_entity.on.value = gnd_pos_entity.name

            if obj_name != "robot":
                obj_entity = cast(Object, entities.get_entities(obj_name))
                obj_entity.goal.value = pos_entity.name

                # Calculate placement approach base locations and inward orientations for the target
                goal_side_data = compute_side_positions_and_orientations(info['position'], info['orientation'], approach_distance)

                place_target_labels = []
                for side_name, data in goal_side_data.items():
                    target_key = f'{obj_entity.name}_place_target_{side_name}'

                    # Apply the adjusted look-at angle poses for the place configurations
                    pose_dict[target_key] = Pose(data['position'], data['orientation'])
                    place_target_labels.append(target_key)

                obj_entity.placeable_from = place_target_labels

    return pose_dict

# Helper to calculate rotated orthogonal side positions and inward-facing orientations
def compute_side_positions_and_orientations(center_pos: List[float], orientation: List[float], dist: float) -> dict:
    cx, cy, _ = center_pos
    # Yaw angle is expected at the 3rd index (index 2) of orientation
    yaw = orientation[2] if len(orientation) > 2 else 0.0
    
    # Local offsets from the cube center
    local_offsets = {
        'front': np.array([dist, 0.0]),
        'back':  np.array([-dist, 0.0]),
        'left':  np.array([0.0, dist]),
        'right': np.array([0.0, -dist])
    }
    
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rotation_matrix = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
    
    world_sides = {}
    for side, local_vector in local_offsets.items():
        # 1. Rotate the displacement vector into world frame coordinates
        world_vector = rotation_matrix.dot(local_vector)
        bx = float(cx + world_vector[0])
        by = float(cy + world_vector[1])
        bz = float(center_pos[2])
        
        # 2. Compute the heading from the base position [bx, by] back to the cube center [cx, cy]
        # atan2(y_target - y_source, x_target - x_source)
        facing_yaw = np.arctan2(cy - by, cx - bx)
        
        # 3. Store both position and the specific inward-facing [roll, pitch, yaw] orientation
        world_sides[side] = {
            'position': [bx, by, bz],
            'orientation': [0.0, 0.0, float(facing_yaw)]
        }
    return world_sides

# def assign_entities_variable_values_and_create_pose_dict(init_config: Dict, goal_config: Dict, entities: Entities) \
#                                                                                                 -> Dict[str, Pose]:
#     pos_counter = 0
#     init_pos_vals = []
#     pose_dict = {}

#     obj_entities = cast(List[Object], entities.get_entities(Object))
#     pos_entities = cast(List[PosEntity], entities.get_entities(PosEntity))
#     gnd_pos_entity = cast(PosEntity, entities.get_entities('g'))
#     gnd_obj_entity = cast(Object, entities.get_entities('g'))

#     for info, obj_entity, pos_entity in zip(init_config.values(), obj_entities[:-2], pos_entities[:-2]):
#         pose = Pose(info['position'], info['orientation'])
#         pose_dict[pos_entity.name] = pose
#         obj_entity.at.value = pos_entity.name
#         obj_entity.on.value = gnd_obj_entity.name
#         obj_entity.reachable_from = [f'{obj_entity.name}_pick_target0']
#         obj_entity.dim = info['size']

#         pose_dict[f'{obj_entity.name}_pick_target0'] = pose
#         pos_entity.occupied_by.value = obj_entity.name
#         pos_entity.on.value = gnd_pos_entity.name
#         pos_entity.clear.value = False

#         init_pos_vals.append(info['position'])
#         pos_counter += 1

#     robot_entity = cast(Robot, entities.get_entities('robot'))
#     robot_entity.at.value = 'r_init_pos'
#     pose_dict[robot_entity.at.value] = Pose(init_config['robot']['position'],
#                                             init_config['robot']['orientation'])

#     for obj_name, info, pos_entity in zip(goal_config.keys(), goal_config.values(), pos_entities[pos_counter+1:-2]):
#         pos_val = info['position']

#         if pos_val not in init_pos_vals:
#             pose = Pose(info['position'], info['orientation'])
#             pose_dict[pos_entity.name] = pose
#             pos_entity.clear.value = True
#             pos_entity.on.value = gnd_pos_entity.name
#             obj_entity.placeable_from = [f'{obj_entity.name}_place_target0']
#             pose_dict[f'{obj_entity.name}_place_target0'] = pose

#             if obj_name != "robot":
#                 obj_entity = cast(Object, entities.get_entities(obj_name))
#                 obj_entity.goal.value = pos_entity.name

#     return pose_dict

def define_goal_state(entities: Entities) -> State:
    goal_state = {}
    obj_entities = cast(List[Object], entities.get_entities(Object))

    for obj_entity in obj_entities:
        if not obj_entity.goal.value:
            continue

        state_key = f"{obj_entity.name}_at"
        state_val = obj_entity.goal.value
        goal_state[state_key] = state_val

    return goal_state

def main():
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"

    world = parse_configs_to_world("basic", problem_config_path)

    for ent in world.entities.entities:
        print(ent.state)

if __name__ == "__main__":
    main()