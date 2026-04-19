from copy import deepcopy
from typing import Dict, List, Type, Tuple, Optional
from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner
from modular_construction_task_planner.eas.parser import parse_configs_to_world
from modular_construction_task_planner.scripts.block_domain import PosEntity, Object, Robot, PickAction, PlaceAction, MoveAction

problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
world = parse_configs_to_world("basic", problem_config_path)
for ent in world.entities.entities:
    print(ent.state)

action_dict = {
    'transit': MoveAction,
    'transport': MoveAction,
    'pick': PickAction,
    'place': PlaceAction
}
planner = OrderedLandmarksPlanner(world, action_dict)
goal_linked_states = planner.run_optimal_planner()

print(f"{len(goal_linked_states)} goal linked states found.")