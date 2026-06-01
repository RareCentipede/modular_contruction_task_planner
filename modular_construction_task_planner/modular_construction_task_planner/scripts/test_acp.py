import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy
from yaml import safe_load

from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.scripts.cost_propagation import (
    spawn_shadow_boxes,
    perform_cost_propagation,
    visualize_cost_propagation
)
from modular_construction_task_planner.scripts.generate_random_configs import generate_random_tamp_configs
from modular_construction_task_planner.scripts.generate_polyhedrals import (
    generate_diced_block,
    compute_base_positions,
)
from path_planner.path_planner_node import GridGraph, OCCUPANCY

def test_acp():
    # --- 1. Generate Random TAMP Configurations ---
    num_objects = 5
    init_dict, goal_dict = generate_random_tamp_configs(num_objects)

    # --- 2. Parse Configurations into World ---
    world = parse_configs_to_world(init_dict, goal_dict)

    # --- 3. Spawn Shadow Boxes for Cost Propagation ---
    shadow_boxes = spawn_shadow_boxes(world)

    # --- 4. Perform Cost Propagation ---
    perform_cost_propagation(world, shadow_boxes)

    # --- 5. Visualize Cost Propagation Results ---
    visualize_cost_propagation(world)

if __name__ == "__main__":
    test_acp()