import rclpy
import time
import numpy as np
import pandas as pd

from copy import deepcopy
from rclpy.node import Node
from yaml import safe_load
from typing import List, Tuple, cast, Dict

from mpnp_interfaces.msg import Plan, TaskAction
from mpnp_interfaces.srv import PlanConstructionTask

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC
from modular_construction_task_planner.eas.core import LinkedState, World, Pose
from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.eas.block_list_parser_world import parse_block_list_to_world
from modular_construction_task_planner.scripts.block_domain import PickAction, PlaceAction, TransitAction, TransportAction, Object
from path_planner.path_planner_node import GridGraph, OCCUPANCY

class ModularConstructionTaskPlanner(Node):
    def __init__(self):
        super().__init__('modular_construction_task_planner')
        self.srv = self.create_service(PlanConstructionTask,
                                       '/tamp/plan_construction_task',
                                       self.plan_construction_task_service)
        self.action_dict = {
            'transit': TransitAction,
            'transport': TransportAction,
            'pick': PickAction,
            'place': PlaceAction
        }
        self.gg: GridGraph | None = None

        self.h = [HEURISTIC.LAZY, HEURISTIC.SIMPLE_COLLISION, HEURISTIC.DILIGENT]
        self.planner_types = ['greedy', 'multi_bound']
        self.columns = ['no.blocks', 'planner_type', 'heuristic', 'planning_time', 'plan_cost', 'pp_cost']
        self.res_path = 'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/results/'

    def plan_construction_task_service(self, request, response):
        try:
            if request.blocks:
                self.get_logger().info("Parsing block list from request to create world representation.")
                # self.get_logger().info(f"Robot initial pose:\n{request.robot_init_pose}")
                world = parse_block_list_to_world(request.blocks, request.robot_init_pose)
                self.gg = GridGraph(list(request.blocks), block_size=0.3)

                # for entity in world.entities.entities:
                    # self.get_logger().info(f"Entity: {entity.name}, State: {entity.state}")
                # self.get_logger().info(f"Pose dict names: {list(world.pose_dict.keys())}")
            else:
                self.get_logger().info("Parsing configuration files to create world representation.")
                world = parse_configs_to_world(request.config_name, request.problem_config_path)

        except Exception as e:
            self.get_logger().error(f"Failed to parse configuration: {e}")
            response.success = False
            response.result = PlanConstructionTask.Response.INVALID_CONFIG
            response.msg = f"Failed to parse configuration: {e}"
            return response

        original_world = deepcopy(world)  # Keep a copy of the original world for logging
        original_gg = deepcopy(self.gg)  # Keep a copy of the original grid graph for logging
        res_pd = pd.DataFrame(columns=self.columns)
        mb_costs_pd = pd.DataFrame(columns=['no.blocks', 'heuristic', 'mb_cost_per_iteration'])
        idx = 0
        mb_idx = 0
        blocks = world.entities.get_entities(Object)
        blocks = cast(List, blocks)
        print(blocks)
        for planner_type in self.planner_types:
            for heuristic in self.h:
                world = deepcopy(original_world)
                gg = deepcopy(original_gg)
                pp_cost = float('inf')
                self.get_logger().info(f"Running {planner_type} planner with {heuristic} heuristic...")
                start_time = time.perf_counter_ns()
                plan, cost, mb_costs = self.run_planner_type_heuristic(world, planner_type, heuristic, gg)
                end_time = time.perf_counter_ns()
                planning_time = end_time - start_time
                if plan:
                    self.get_logger().info(f"Plan found with {planner_type} planner and {heuristic} heuristic. Cost: {cost:.2f}. Time: {planning_time/1e9} seconds.")
                    pp_cost = self.compute_full_nav_cost(plan, world.pose_dict)
                    self.get_logger().info(f"Navigation cost: {pp_cost:.2f}")
                else:
                    self.get_logger().warning(f"No plan found with {planner_type} planner and {heuristic} heuristic.")

                res_row = [len(blocks)-2, planner_type, heuristic, planning_time/1e9, cost, pp_cost]
                res_pd.loc[idx] = res_row
                res_pd.to_csv(f"{self.res_path}{len(blocks)-2}_{planner_type}_{heuristic}.csv", index=False)
                idx += 1

                if planner_type == 'multi_bound':
                    for mb_cost in mb_costs:
                        mb_row = [len(blocks)-2, heuristic, mb_cost]
                        mb_costs_pd.loc[mb_idx] = mb_row
                        mb_idx += 1
                    mb_costs_pd.to_csv(f"{self.res_path}{len(blocks)-2}_{planner_type}_{heuristic}_mb_costs.csv", index=False)

        planner_type = 'multi_bound'
        world = deepcopy(original_world)
        gg = deepcopy(original_gg)
        pp_cost = float('inf')
        self.get_logger().info(f"Running {planner_type} planner with {heuristic} heuristic...")
        start_time = time.perf_counter_ns()
        plan, cost, mb_costs = self.run_planner_type_heuristic(world, planner_type, heuristic, gg, mixed=True)
        end_time = time.perf_counter_ns()
        planning_time = end_time - start_time
        if plan:
            self.get_logger().info(f"Plan found with {planner_type} planner and {heuristic} heuristic. Cost: {cost:.2f}. Time: {planning_time/1e9} seconds.")
            pp_cost = self.compute_full_nav_cost(plan, world.pose_dict)
            self.get_logger().info(f"Navigation cost: {pp_cost:.2f}")
        else:
            self.get_logger().warning(f"No plan found with {planner_type} planner and {heuristic} heuristic.")

        res_row = [len(blocks)-2, planner_type, 'mixed', planning_time/1e9, cost, pp_cost]
        res_pd.loc[idx] = res_row
        res_pd.to_csv(f"{self.res_path}{len(blocks)-2}_{planner_type}_mixed.csv", index=False)

        for mb_cost in mb_costs:
            mb_row = [len(blocks)-2, heuristic, mb_cost]
            mb_costs_pd.loc[mb_idx] = mb_row
            mb_idx += 1
        mb_costs_pd.to_csv(f"{self.res_path}{len(blocks)-2}_{planner_type}_mixed_mb_costs.csv", index=False)

        best_plan = plan
        response.plan = self.format_plan(best_plan) if best_plan else Plan()
        response.success = True
        response.result = PlanConstructionTask.Response.SUCCESS
        response.msg = "Plan found successfully."

        return response

    def run_planner_type_heuristic(self, world: World, planner_type: str, heuristic: HEURISTIC,
                                   gg: GridGraph | None = None, mixed: bool = False):
        planner = OrderedLandmarksPlanner(world, self.action_dict, gg)
        plan = []
        cost = 0.0
    
        if planner_type == 'greedy':
            goal_state = planner.run_heuristic_planner(heuristic)
        elif planner_type == 'multi_bound':
            if not mixed:
                goal_state = planner.run_multi_bound_planner(heuristic, heuristic)
            else:
                goal_state = planner.run_multi_bound_planner(HEURISTIC.LAZY, HEURISTIC.DILIGENT)
        else:
            raise ValueError(f"Unknown planner type: {planner_type}")

        if goal_state:
            plan, cost = self.retrace_best_plan(goal_state)

        return plan, cost, planner.mb_costs

    @staticmethod
    def retrace_best_plan(goal_linked_states: List[LinkedState] | LinkedState) -> Tuple[List[Tuple[str, Tuple[str, ...]]], float]:
        # Backtrack to get all the plans with total costs
        plans = []
        if isinstance(goal_linked_states, LinkedState):
            goal_linked_states = [goal_linked_states]

        for goal_linked_state in goal_linked_states:
            plan = []
            total_cost = 0.0
            current_linked_state = goal_linked_state
            parent = current_linked_state.parent
            while parent is not None:
                plan.append(current_linked_state.action_from_parent)  # (action_name, involved_entities)
                current_linked_state = parent[1]
                parent = current_linked_state.parent

            total_cost = goal_linked_state.cost
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
            task_action.host, task_action.source, task_action.target = params[0], params[1], params[2]
            actions.append(task_action)

        task_plan.actions = actions
        return task_plan

    def compute_full_nav_cost(self, plan: List[Tuple[str, Tuple[str, ...]]], pose_dict: Dict[str, Pose]) -> float:
        self.gg = cast(GridGraph, self.gg)  # Type hint for better code completion
        pp_cost = 0.0
        for action_name, params in plan:
            match action_name:
                case 'pick':
                    obj_pos = pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.FREE)
                case 'place':
                    obj_pos = pose_dict[params[2]].position[:2]
                    self.gg.update_block_move(obj_pos, OCCUPANCY.OCCUPIED)
                case 'transit' | 'transport':
                    start_pos = pose_dict[params[1]].position[:2]
                    target_pos = pose_dict[params[2]].position[:2]
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