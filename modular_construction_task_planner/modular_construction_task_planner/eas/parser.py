import numpy as np

from yaml import safe_load
from typing import Dict, List, Tuple, cast
from scipy.spatial import cKDTree
from modular_construction_task_planner.eas.core import (
    load_domains,
    Entities,
    World,
    Pose
)
from modular_construction_task_planner.scripts.block_domain import (
    Object,
    PosEntity,
    Robot
)

def parse_configs_to_world(config_name: str, problem_config_path: str) -> World:
    init_path = problem_config_path + config_name + "/init.yaml"
    goal_path = problem_config_path + config_name + "/goal.yaml"

    with open(init_path, 'r') as f:
        init_config = safe_load(f)
        f.close()

    with open(goal_path, 'r') as f:
        goal_config = safe_load(f)
        f.close()

    domains = create_domains(init_config, goal_config)
    load_domains(domains)
    entities = create_entities(domains)
    pose_dict = assign_entities_variable_values_and_create_pose_dict(init_config, goal_config, entities)
    world = World(entities, pose_dict=pose_dict)
    goal_entities_dict = {
        Object: ['at']
    }
    world.goal_entities_dict = goal_entities_dict

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
    bool_var_domain = (True, False)

    for obj_name in init_config.keys():
        pos_name = 'p' + str(idx)
        pos_var_domain.append(pos_name)

        if obj_name != "robot":
            block_var_domain.append(obj_name)

        idx += 1

    for _ in goal_config.values():
        pos_name = 'p' + str(idx)
        pos_var_domain.append(pos_name)
        idx += 1

    pos_var_domain.append('g')
    pos_var_domain.append('')
    block_var_domain.append('g')
    block_var_domain.append('')
    domains = {
        'pos': tuple(pos_var_domain),
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
    entities.append(Robot('robot'))

    return Entities(entities)

def assign_entities_variable_values_and_create_pose_dict(init_config: Dict, goal_config: Dict, entities: Entities) \
                                                                                                -> Dict[str, Pose]:
    pos_counter = 0
    init_pos_vals = []
    pose_dict = {}

    obj_entities = cast(List[Object], entities.get_entities(Object))
    pos_entities = cast(List[PosEntity], entities.get_entities(PosEntity))
    gnd_pos_entity = cast(PosEntity, entities.get_entities('g'))
    gnd_obj_entity = cast(Object, entities.get_entities('g'))

    for info, obj_entity, pos_entity in zip(init_config.values(), obj_entities[:-2], pos_entities[:-2]):
        pose = Pose(info['position'], info['orientation'])
        pose_dict[pos_entity.name] = pose
        obj_entity.at.value = pos_entity.name
        obj_entity.on.value = gnd_obj_entity.name

        pos_entity.occupied_by.value = obj_entity.name
        pos_entity.on.value = gnd_pos_entity.name
        pos_entity.clear.value = False

        init_pos_vals.append(info['position'])
        pos_counter += 1

    robot_entity = cast(Robot, entities.get_entities('robot'))
    robot_entity.at.value = pos_entities[pos_counter+1].name

    for obj_name, info, pos_entity in zip(goal_config.keys(), goal_config.values(), pos_entities[pos_counter:-2]):
        pos_val = info['position']

        if pos_val not in init_pos_vals:
            pose = Pose(info['position'], info['orientation'])
            pose_dict[pos_entity.name] = pose
            pos_entity.clear.value = True
            pos_entity.on.value = gnd_pos_entity.name

            if obj_name != "robot":
                obj_entity = cast(Object, entities.get_entities(obj_name))
                obj_entity.goal.value = pos_entity.name

    return pose_dict

problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"

world = parse_configs_to_world("basic", problem_config_path)

for ent in world.entities.entities:
    print(ent.state)