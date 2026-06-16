import os
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import random

from tqdm import tqdm
from copy import deepcopy
from typing import List, Dict, cast
from yaml import safe_load
from enum import Enum

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC, Pose
from modular_construction_task_planner.modular_construction_task_planner import ModularConstructionTaskPlanner
from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.eas.core import World
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
from modular_construction_task_planner.scripts.block_domain import (
    PickAction,
    PlaceAction,
    TransitAction,
    TransportAction,
    Object
)
from mpnp_interfaces.msg import Block

PLANNER_TYPE = Enum('PLANNER_TYPE', 'HEURISTIC MULTI_BOUND MULTI_BOUND_G')

def parse_objects_to_blocks(objs: List[Object], pose_dict: Dict[str, Pose]) -> List[Block]:
    blocks = []
    for obj in objs:
        block = Block()
        block.name = obj.name

        if not obj.at.value:
            continue

        init_pose = pose_dict[obj.at.value]

        block.init_pose.pose.position.x = init_pose.position[0]
        block.init_pose.pose.position.y = init_pose.position[1]
        block.init_pose.pose.position.z = init_pose.position[2]

        if obj.goal.value:
            goal_pose = pose_dict[obj.goal.value]
            block.goal_pose.pose.position.x = goal_pose.position[0]
            block.goal_pose.pose.position.y = goal_pose.position[1]
            block.goal_pose.pose.position.z = goal_pose.position[2]
        blocks.append(block)
    return blocks

def randomize_initial_staging(goal_data: Dict, workspace_limit: float = 15.0, grid_snap: float = 0.5, 
                              safety_margin: float = 1.2, max_attempts: int = 2000) -> Dict:
    """
    Reads a goal configuration YAML, extracts the blocks, and randomizes their initial 
    positions flat on the floor strictly within the -Y staging zone.
    
    Args:
        goal_yaml_path: Path to the target structure file (e.g., 'goal.yaml').
        workspace_limit: Maximum grid bounds along the X and Y axes.
        grid_snap: Step resolution to round coordinates to (e.g., 0.5m increments).
        safety_margin: Minimum clear center-to-center distance required between blocks to prevent overlap.
        max_attempts: Loop ceiling before raising a placement saturation exception.
    """
    init_positions = {'robot': {
        'position': [0.0, 0.0, 0.0],
        'orientation': [0.0, 0.0, 0.0],
        'size': [0.0, 0.0, 0.0]
    }}

    # Define bounds for the staging area floor
    # We leave a 1.0m boundary buffer relative to the workspace limits so blocks don't clip walls
    min_x, max_x = -workspace_limit + 1.0, workspace_limit - 1.0
    min_y, max_y = -workspace_limit + 1.0, -1.0  # Strictly below the Y=0 construction border

    # 2. Sequential placement loop with geometric collision checking
    for block_name, block_attributes in goal_data.items():
        size = block_attributes.get('size', [1.0, 1.0, 1.0])
        z_baseline = size[2] / 2.0  # Center-of-mass elevation for a block resting flat on the ground

        placed = False
        attempts = 0

        while not placed:
            if attempts > max_attempts:
                raise RuntimeError(
                    f"Workspace saturation reached at '{block_name}'! "
                    f"The staging zone is too packed. Increase your workspace_limit or reduce safety_margin."
                )

            # Generate random coordinates within the -Y staging domain
            rand_x = round(random.uniform(min_x, max_x) / grid_snap) * grid_snap
            rand_y = round(random.uniform(min_y, max_y) / grid_snap) * grid_snap
            rand_yaw = random.uniform(0, 2 * np.pi)

            # Intersection/Overlap test against previously committed staging nodes
            collision = False
            for existing_name, existing_data in init_positions.items():
                ex, ey, _ = existing_data['position']
                
                # Check distance buffer on the flat ground plane
                if abs(ex - rand_x) < safety_margin and abs(ey - rand_y) < safety_margin:
                    collision = True
                    break

            # If the spot is clear, save it matching your schema
            if not collision:
                init_positions[block_name] = {
                    'position': [float(rand_x), float(rand_y), float(z_baseline)],
                    'orientation': [0.0, 0.0, float(rand_yaw)],
                    'size': [float(i) for i in size]
                }
                placed = True

            attempts += 1

    return init_positions

def test_planner(world: World, planner_type: PLANNER_TYPE, heuristic: HEURISTIC):
    # --- 1. Generate Random TAMP Configurations ---
    mb_cost_hist = []

    action_dict = {
            'transit': TransitAction,
            'transport': TransportAction,
            'pick': PickAction,
            'place': PlaceAction
        }

    obj_list = world.entities.get_entities(Object)
    obj_list = cast(List[Object], obj_list)
    blocks = parse_objects_to_blocks(obj_list, world.pose_dict)
    block_size = 1.0

    grid_graph = GridGraph(blocks, block_size)
    planner = OrderedLandmarksPlanner(world, action_dict, grid_graph)

    start_time = time.perf_counter_ns()
    try:
        match planner_type:
            case PLANNER_TYPE.MULTI_BOUND:
                goal_state = planner.run_multi_bound_planner()
                mb_cost_hist = planner.mb_costs
            case PLANNER_TYPE.MULTI_BOUND_G:
                goal_state = planner.run_multi_bound_planner_g()
                mb_cost_hist = planner.mb_costs
            case PLANNER_TYPE.HEURISTIC:
                goal_state = planner.run_heuristic_planner(heuristic)
            case _:
                raise ValueError("Invalid planner type specified.")
    except Exception as e:
        print(f"Planner encountered an error: {e}")
        return np.nan, np.nan, np.nan, np.nan, mb_cost_hist

    end_time = time.perf_counter_ns()
    time_taken = (end_time - start_time) / 1e9  # Convert

    if not goal_state:
        print("No plan found for this trial.")
        return np.nan, np.nan, np.nan, time_taken, mb_cost_hist

    plan, cost = ModularConstructionTaskPlanner.retrace_best_plan(goal_state)
    full_cost = planner.compute_full_nav_cost(plan)
    return cost, full_cost, planner.state_counter, time_taken, mb_cost_hist

if __name__ == "__main__":
    config_path = "src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/configs/"
    problems = ['box', 'triangle', 'circle']
    trials = 10
    # problems = ['box']
    res_df_col = ['planner', 'heuristic', 'problem', 'trial_num', 'est_cost', 'cost', 'states_explored', 'time_taken']
    res_df = pd.DataFrame(columns=res_df_col)
    res_path = 'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/results/'

    mb_res_col = ['mb_type', 'problem', 'trial_num', 'iter', 'cost']
    mb_res_df = pd.DataFrame(columns=mb_res_col)

    settings = [
            (PLANNER_TYPE.HEURISTIC, HEURISTIC.LAZY),
            (PLANNER_TYPE.HEURISTIC, HEURISTIC.DILIGENT),
            (PLANNER_TYPE.HEURISTIC, HEURISTIC.ANTICIPATORY),
            (PLANNER_TYPE.HEURISTIC, HEURISTIC.ANTICIPATORY_ONCE),
            (PLANNER_TYPE.HEURISTIC, HEURISTIC.ANTICIPATORY_ONCE_DISCOUNT),
            # (PLANNER_TYPE.MULTI_BOUND, HEURISTIC.MIXED),
            (PLANNER_TYPE.MULTI_BOUND_G, HEURISTIC.MIXED_G)
        ]

    idx = 0
    mb_idx = 0

    for problem in problems:
        with open(f"{config_path}/{problem}/goal.yaml", "r") as f:
            goal_dict = safe_load(f)
        est_costs = []
        full_costs = []
        states_explored_hist = []
        times = []

        for trial in range(trials):
            print(f"\n--- Trial {trial+1}/{trials} ---")
            init_dict = randomize_initial_staging(goal_dict)
            world = parse_configs_to_world(init_dict, goal_dict)
            original_world = deepcopy(world)

            for planner_type, heuristic in tqdm(settings):
                tqdm.write(f"\nTesting {planner_type.name} + {heuristic.name} with {problem} objects...")
                results = test_planner(world, planner_type, heuristic)
                est_cost, full_cost, states_explored, time_taken, mb_cost_hist = results
                res_row = [planner_type.name, heuristic.name, problem, trial, est_cost, full_cost, states_explored, time_taken]
                res_df.loc[idx] = res_row
                idx += 1

                if planner_type != PLANNER_TYPE.HEURISTIC:
                    for iter_num, cost in enumerate(mb_cost_hist):
                        mb_res_row = [planner_type.name, problem, trial, iter_num, cost]
                        mb_res_df.loc[mb_idx] = mb_res_row
                        mb_idx += 1

                world = deepcopy(original_world)

                if heuristic == HEURISTIC.ANTICIPATORY_ONCE or heuristic == HEURISTIC.ANTICIPATORY_ONCE_DISCOUNT:
                    # Reset world to original state for next planner test
                    shadow_boxes = spawn_shadow_boxes(world)
                    perform_cost_propagation(world, shadow_boxes)

        res_df.to_csv(res_path + 'planner_designed_structs.csv', index=False)
        mb_res_df.to_csv(res_path + 'planner_mb_planner_designed_structs_costs.csv', index=False)