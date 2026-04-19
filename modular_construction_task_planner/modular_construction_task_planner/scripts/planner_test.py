from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner
from modular_construction_task_planner.eas.parser import parse_configs_to_world
from modular_construction_task_planner.scripts.block_domain import PickAction, PlaceAction, MoveAction, PosEntity

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

for plan in plans:
    print(f"Plan with total cost {plan[1]}:")
    for action in plan[0]:
        print(f"  {action}")