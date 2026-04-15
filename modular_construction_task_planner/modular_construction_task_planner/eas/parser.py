import numpy as np

from yaml import safe_load
from copy import deepcopy
from typing import Dict, List, Type, Tuple, Optional
from scipy.spatial import cKDTree
from modular_construction_task_planner.eas.core import (
    load_domains,
    State,
    LinkedState,
    Entity,
    Entities,
    World,
    Pose
)
from modular_construction_task_planner.scripts.block_domain import (
    Object, At, AtTop, OnBlock, BelowBlock, Supported, Goal,
    PosEntity, Clear, OccupiedBy, OnPose, BelowPose,
    Robot, GripperEmpty, Holding,
)

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

def create_world(init_config: Dict, goal_config: Dict):
    idx = 1
    entities = []
    bool_var_domain = (True, False)
    block_var_domain = []
    pos_var_domain = []
    init_pos_vals = []
    pose_dict = {}

    for obj_name, info in init_config.items():
        pos_name = 'p' + str(idx)
        pos_var_domain.append(pos_name)

        if obj_name != "robot":
            block_var_domain.append(obj_name)

        init_pos_vals.append(info['position'])
        pose = Pose(position=info['position'],
                    orientation=info['orientation'])
        pose_dict[pos_name] = pose
        idx += 1

    for info in goal_config.values():
        pos_val = info['position']
        pos_name = 'p' + str(idx)
        unique_pos = True

        for init_pos_val in init_pos_vals:
            if pos_val == init_pos_val:
                unique_pos = False
                break

        if unique_pos:
            pos_var_domain.append(pos_name)
            pose = Pose(position=pos_val, orientation=info['orientation'])
            pose_dict[pos_name] = pose
            idx += 1

    domains = {
        'pos': tuple(pos_var_domain),
        'block': tuple(block_var_domain),
        'bool': bool_var_domain
    }
    load_domains(domains)

    for pos in pos_var_domain:
        pos_entity = PosEntity(pos)
        entities.append(pos_entity)

    for obj in block_var_domain:
        block = Object(obj)
        entities.append(block)

    robot = Robot('robot')
    entities.append(robot)
    world = World(Entities(entities))

    return world

    # if obj_name != "robot":
    #     block = Object(obj_name)
    #     block.at.value = pos_name
    #     pos_entity.occupied_by.value = obj_name
    #     entities.append(block)
    #     entities.append(pos_entity)

    #     block_var_domain.append(obj_name)
    # else:
    #     robot = Robot(obj_name)
    #     robot.at.value = pos_name
    #     pos_entity.occupied_by.value = obj_name
    #     entities.append(robot)
    #     entities.append(pos_entity)

problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"

init_config, goal_config = load_configs_to_dict("basic", problem_config_path)
world = create_world(init_config, goal_config)

print(world)