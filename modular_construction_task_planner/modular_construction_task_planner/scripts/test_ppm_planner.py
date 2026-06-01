import matplotlib.pyplot as plt
import numpy as np

from tqdm import tqdm
from copy import deepcopy
from typing import List, Dict, cast
from yaml import safe_load

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC, Pose
from modular_construction_task_planner.modular_construction_task_planner import ModularConstructionTaskPlanner
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
from modular_construction_task_planner.scripts.block_domain import (
    PickAction,
    PlaceAction,
    TransitAction,
    TransportAction,
    Object
)
from mpnp_interfaces.msg import Block

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

def test_planner():
    # --- 1. Generate Random TAMP Configurations ---
    num_objects_list = [5, 10, 20, 30, 50]
    num_trials = 10
    ant_costs = []
    lazy_costs = []

    for num_objects in num_objects_list:
        total_ant_cost = 0
        total_lazy_cost = 0
        tqdm.write(f"\nTesting with {num_objects} objects...")
        for trial in tqdm(range(num_trials)):
            print(f"\n--- Trial {trial+1}/{num_trials} ---")
            init_dict, goal_dict = generate_random_tamp_configs(num_objects)

            # --- 2. Parse Configurations into World ---
            world = parse_configs_to_world(init_dict, goal_dict)

            # --- 3. Spawn Shadow Boxes for Cost Propagation ---
            shadow_boxes = spawn_shadow_boxes(world)

            # --- 4. Perform Cost Propagation ---
            perform_cost_propagation(world, shadow_boxes)

            action_dict = {
                    'transit': TransitAction,
                    'transport': TransportAction,
                    'pick': PickAction,
                    'place': PlaceAction
                }
            obj_list = world.entities.get_entities(Object)
            obj_list = cast(List[Object], obj_list)
            blocks = parse_objects_to_blocks(obj_list, world.pose_dict)
            block_size = 0.3
            grid_graph = GridGraph(blocks, block_size)

            planner = OrderedLandmarksPlanner(world, action_dict, grid_graph)

            original_world = deepcopy(world)
            original_gg = deepcopy(grid_graph)

            goal_state = planner.run_heuristic_planner(HEURISTIC.ANTICIPATORY)
            plan, cost = ModularConstructionTaskPlanner.retrace_best_plan(goal_state)
            ant_cost = planner.compute_full_nav_cost(plan)

            planner = OrderedLandmarksPlanner(original_world, action_dict, original_gg)
            goal_state = planner.run_heuristic_planner(HEURISTIC.LAZY)
            plan, cost = ModularConstructionTaskPlanner.retrace_best_plan(goal_state)
            lazy_cost = planner.compute_full_nav_cost(plan)

            print(f"Anticipatory Heuristic Cost: {ant_cost}")
            print(f"Lazy Heuristic Cost: {lazy_cost}")

            total_ant_cost += ant_cost
            total_lazy_cost += lazy_cost

        ant_costs.append(total_ant_cost / num_trials)
        lazy_costs.append(total_lazy_cost / num_trials)

        print(f"\nAverage Anticipatory Heuristic Cost over {num_trials} trials: {total_ant_cost / num_trials:.2f}")
        print(f"Average Lazy Heuristic Cost over {num_trials} trials: {total_lazy_cost / num_trials:.2f}")

    plt.scatter(num_objects_list, ant_costs, label='Anticipatory Heuristic Cost', color='blue')
    plt.scatter(num_objects_list, lazy_costs, label='Lazy Heuristic Cost', color='red')
    plt.plot(num_objects_list, ant_costs, label='Average Anticipatory Cost', color='blue', linestyle='--')
    plt.plot(num_objects_list, lazy_costs, label='Average Lazy Cost', color='red', linestyle='--')
    plt.xlabel('Number of Objects')
    plt.ylabel('Navigation Cost')
    plt.title('Navigation Costs for Anticipatory vs Lazy Heuristics')
    plt.legend()
    plt.grid()
    plt.show()

    # --- 5. Visualize Cost Propagation Results ---
    # visualize_cost_propagation(world)

if __name__ == "__main__":
    test_planner()