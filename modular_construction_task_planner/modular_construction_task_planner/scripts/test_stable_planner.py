import matplotlib.pyplot as plt

from yaml import safe_load

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner
from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.scripts.block_domain import PickAction, PlaceAction, TransitAction, TransportAction
from modular_construction_task_planner.scripts.stability import (
    visualize_goal_structure,
    create_support_relation_graph,
    visualize_support_node_graph,
    animate_construction_sequence,
    find_feasible_block_sequence
)
from modular_construction_task_planner.scripts.block_sequence_animator import TrimeshBlockAnimator

def main():
    goal_linked_state = None
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    problem_name = "scaffolding_tower"
    world = parse_configs_to_world(problem_name, problem_config_path)
    for ent in world.entities.entities:
        print(f"{ent.name}: {ent.state}")

    init_config = safe_load(open(f"{problem_config_path}/{problem_name}/init.yaml", 'r'))
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    colors = visualize_goal_structure(goal_config, show=False)
    ground_mesh, support_graph = create_support_relation_graph(world)
    seq = find_feasible_block_sequence(support_graph)
    visualize_support_node_graph(support_graph, colors=colors, show=False)
    if not seq:
        print("No valid block placement sequence found due to cyclic support relations.")
        return
    print(f"Feasible block placement sequence found: {seq}")

    action_dict = {
        'transit': TransitAction,
        'transport': TransportAction,
        'pick': PickAction,
        'place': PlaceAction
    }

    planner = OrderedLandmarksPlanner(world, action_dict)
    goal_linked_state = planner.run_stable_planner(support_graph, ground_mesh)

    if goal_linked_state:
        print(f"Goal linked state found! :)")
        plan, _ = planner.retrace_best_plan(goal_linked_state)
        print(f"Plan found with {len(plan)} actions:")
        block_sequence = []
        for action in plan:
            if action[0] == 'pick':
                print(action[1][1])
                block_sequence.append(action[1][1])

        animator = TrimeshBlockAnimator(init_config, goal_config, block_sequence, colors)
        # animator.run()
        animator.save_to_video(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}.mp4',
                               fps=60)
        # construction_animation = animate_construction_sequence(init_config, goal_config, block_sequence, colors, interval=500)
        # construction_animation.save(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}_full.gif', writer='pillow')
    else:
        print(f"Goal linked state not found :(.")

    plt.show()

if __name__ == "__main__":
    main()