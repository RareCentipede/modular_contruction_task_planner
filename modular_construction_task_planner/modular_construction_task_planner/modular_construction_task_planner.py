import rclpy
import numpy as np

from copy import deepcopy
from rclpy.node import Node
from yaml import safe_load
from typing import List, Tuple, cast

from mpnp_interfaces.msg import Plan, TaskAction
from mpnp_interfaces.srv import PlanConstructionTask

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC
from modular_construction_task_planner.eas.core import LinkedState
from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.eas.block_list_parser_world import parse_block_list_to_world
from modular_construction_task_planner.scripts.block_domain import PickAction, PlaceAction, MoveAction
from path_planner.path_planner_node import GridGraph, OCCUPANCY

class ModularConstructionTaskPlanner(Node):
    def __init__(self):
        super().__init__('modular_construction_task_planner')
        self.srv = self.create_service(PlanConstructionTask,
                                       '/tamp/plan_construction_task',
                                       self.plan_construction_task_service)
        self.action_dict = {
            'transit': MoveAction,
            'transport': MoveAction,
            'pick': PickAction,
            'place': PlaceAction
        }
        self.gg: GridGraph | None = None

    def plan_construction_task_service(self, request, response):
        try:
            if request.blocks:
                self.get_logger().info("Parsing block list from request to create world representation.")
                self.get_logger().info(f"Robot initial pose:\n{request.robot_init_pose}")
                world = parse_block_list_to_world(request.blocks, request.robot_init_pose)
                original_world = deepcopy(world)  # Keep a copy of the original world for logging
                self.gg = GridGraph(list(request.blocks), block_size=0.3)

                for entity in world.entities.entities:
                    self.get_logger().info(f"Entity: {entity.name}, State: {entity.state}")
                self.get_logger().info(f"Pose dict names: {list(world.pose_dict.keys())}")
            else:
                self.get_logger().info("Parsing configuration files to create world representation.")
                world = parse_configs_to_world(request.config_name, request.problem_config_path)

        except Exception as e:
            self.get_logger().error(f"Failed to parse configuration: {e}")
            response.success = False
            response.result = PlanConstructionTask.Response.INVALID_CONFIG
            response.msg = f"Failed to parse configuration: {e}"
            return response

        planner = OrderedLandmarksPlanner(world, self.action_dict)
        h = HEURISTIC.LAZY
        lazy_greedy_heuristic_goal_state = planner.run_heuristic_planner(heuristic=h)
        lazy_greedy_plan, heuristic_cost = self.retract_best_plan(lazy_greedy_heuristic_goal_state)
        self.get_logger().info(f"Heuristic {h.name} plan with total cost {heuristic_cost} found.")
        best_plan = lazy_greedy_plan

        full_nav_cost = self.compute_full_nav_cost(best_plan, planner)
        self.get_logger().info(f"Total path planning cost for the plan: {full_nav_cost:.2f}")

        response.plan = self.format_plan(best_plan)
        response.success = True
        response.result = PlanConstructionTask.Response.SUCCESS
        response.msg = "Plan found successfully."

        return response

    def retract_best_plan(self, goal_linked_states: List[LinkedState]) -> Tuple[List[Tuple[str, Tuple[str, ...]]], float]:
        # Backtrack to get all the plans with total costs
        plans = []
        for goal_linked_state in goal_linked_states:
            plan = []
            total_cost = 0.0
            current_linked_state = goal_linked_state
            parent = current_linked_state.parent
            while parent is not None:
                plan.append(current_linked_state.action_from_parent)  # (action_name, involved_entities)
                current_linked_state = parent[1]
                total_cost += current_linked_state.cost
                parent = current_linked_state.parent

            plan.reverse()  # Reverse to get the correct order from initial state to goal
            plans.append((plan, total_cost))

        best_plan = min(plans, key=lambda plan: plan[1])  # Get the plan with the lowest total cost
        return best_plan

    def format_plan(self, plan: List[Tuple[str, Tuple[str, ...]]]) -> Plan:
        task_plan = Plan()
        actions = []
        for action_name, params in plan:
            task_action = TaskAction()
            task_action.action_name = action_name
            task_action.host, task_action.source, task_action.target = params
            actions.append(task_action)

        task_plan.actions = actions
        return task_plan

    def compute_full_nav_cost(self, plan: List[Tuple[str, Tuple[str, ...]]], planner: OrderedLandmarksPlanner) -> float:
        self.gg = cast(GridGraph, self.gg)  # Type hint for better code completion
        pp_cost = 0.0
        for action_name, params in plan:
            match action_name:
                case 'pick':
                    obj_pos = planner.world.pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                case 'place':
                    obj_pos = planner.world.pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)
                case 'transit' | 'transport':
                    start_pos = planner.world.pose_dict[params[1]].position[:2]
                    target_pos = planner.world.pose_dict[params[2]].position[:2]
                    path = self.gg.plan(start_pos, target_pos)
                    if path.size == 0:
                        self.get_logger().error(f"No path found for action {action_name} from {start_pos} to {target_pos}.")
                    else:
                        path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                        pp_cost += path_length
                        self.get_logger().info(f"Path found for action {action_name} from {start_pos} to {target_pos} "
                                               f"with length {path_length:.2f}.")
                        # Optionally, visualize the path here using RViz or another tool
                case _:
                    self.get_logger().warning(f"Unknown action {action_name} in plan. Skipping...")

        return pp_cost

def main():
    rclpy.init()
    task_planner_node = ModularConstructionTaskPlanner()
    try:
        rclpy.spin(task_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        task_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()