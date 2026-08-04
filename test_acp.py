import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy
from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.cost_propagation import (
    spawn_shadow_boxes,
    perform_cost_propagation,
    visualize_cost_propagation
)
from modular_construction_task_planner.generate_random_configs import generate_random_tamp_configs
from modular_construction_task_planner.generate_polyhedrals import (
    generate_diced_block,
    compute_base_positions,
)
from path_planner.path_planner_node import GridGraph, OCCUPANCY

def test_acp(config_path: str = '', problem_name: str = '', vis: bool = True):
    # --- 1. Generate Random TAMP Configurations or load pre-defined configs---
    if config_path == '' or problem_name == '':
        num_objects = 5
        init_dict, goal_dict = generate_random_tamp_configs(num_objects)
    else:
        # Load pre-defined configs
        with open(f"{config_path}/{problem_name}/init.yaml", "r") as f:
            init_dict = safe_load(f)
        with open(f"{config_path}/{problem_name}/goal.yaml", "r") as f:
            goal_dict = safe_load(f)

    # --- 2. Parse Configurations into World ---
    world = parse_configs_to_world(init_dict, goal_dict)

    # --- 3. Spawn Shadow Boxes for Cost Propagation ---
    shadow_boxes = spawn_shadow_boxes(world)

    # --- 4. Perform Cost Propagation ---
    perform_cost_propagation(world, shadow_boxes)

    # --- 5. Visualize Cost Propagation Results ---
    visualize_cost_propagation(world, problem_name)

if __name__ == "__main__":
    config_path = "src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/configs/"
    problems = [
        # 'two_obj_real_col',
        # 'two_obj_shadow_col',
        # 'three_obj_all_real',
        # 'three_obj_all_shadow',
        # 'three_obj_real_shadow_diff',
        # 'three_obj_real_shadow_same',
        'box'
    ]

    for problem_name in problems:
        test_acp(config_path, problem_name)