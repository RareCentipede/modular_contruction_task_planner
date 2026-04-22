import rclpy

from rclpy.node import Node
from yaml import safe_load
from typing import List, Tuple

from mpnp_interfaces.msg import Plan, TaskAction
from mpnp_interfaces.srv import PlanConstructionTask

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner
from modular_construction_task_planner.eas.core import LinkedState
from modular_construction_task_planner.eas.parser import parse_configs_to_world, parse_block_list_to_world
from modular_construction_task_planner.scripts.block_domain import PickAction, PlaceAction, MoveAction

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

    def plan_construction_task_service(self, request, response):
        try:
            if request.blocks:
                self.get_logger().info("Parsing block list from request to create world representation.")
                world = parse_block_list_to_world(request.blocks, request.robot_init_pose)
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
        goal_linked_states = planner.run_optimal_planner()

        if len(goal_linked_states) == 0:
            self.get_logger().warn("No plan found for the given configuration.")
            response.success = False
            response.result = PlanConstructionTask.Response.PLANNING_FAILED
            response.msg = "No plan found for the given configuration."
            return response

        self.get_logger().info(f"{len(goal_linked_states)} goal linked states found.")

        best_plan, total_cost = self.retract_best_plan(goal_linked_states)
        self.get_logger().info(f"Best plan with total cost {total_cost} found.")

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