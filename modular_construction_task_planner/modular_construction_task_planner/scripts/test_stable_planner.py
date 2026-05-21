import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy
from yaml import safe_load

from modular_construction_task_planner.scripts.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC
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
from path_planner.path_planner_node import GridGraph, OCCUPANCY

def main():
    animate = False
    show = True
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
        print("No valid block placement sequence found due to invalid supports.")
        return
    print(f"Feasible block placement sequence found: {seq}")

    action_dict = {
        'transit': TransitAction,
        'transport': TransportAction,
        'pick': PickAction,
        'place': PlaceAction
    }

    supp_results = {}
    block_sequences = []
    h = HEURISTIC.STABLE
    planner = OrderedLandmarksPlanner(world, action_dict)
    goal_linked_state = planner.run_stable_planner(support_graph, ground_mesh, h)
    # goal_linked_state = planner.run_heuristic_planner(h)

    if goal_linked_state:
        plan, cost = planner.retrace_best_plan(goal_linked_state)
        print(f"Goal linked state found using {h.name}! Total cost: {cost}")
        lazy_nav_cost = planner.compute_lazy_nav_cost(plan)
        print(f"Lazy navigation cost: {lazy_nav_cost}")
        # full_nav_cost = planner.compute_full_nav_cost(plan)
        block_sequence = []
        for action in plan:
            if action[0] == 'pick':
                print(action[1][1])
                block_sequence.append(action[1][1])
        supp_res = planner.unpack_stability_results(goal_linked_state)
        supp_results[h.name] = supp_res
        block_sequences.append(block_sequence)

        if animate:
            animator = TrimeshBlockAnimator(init_config, goal_config, block_sequence, colors)
            animator.run()

    planner.reset()
    h = HEURISTIC.STABLE_NAV
    goal_linked_state = planner.run_stable_planner(support_graph, ground_mesh, h)
    # goal_linked_state = planner.run_heuristic_planner(h)

    if goal_linked_state:
        plan, cost = planner.retrace_best_plan(goal_linked_state)
        print(f"Goal linked state found using {h.name}! Total cost: {cost}")
        lazy_nav_cost = planner.compute_lazy_nav_cost(plan)
        print(f"Lazy navigation cost: {lazy_nav_cost}")
        # full_nav_cost = planner.compute_full_nav_cost(plan)
        block_sequence = []
        for action in plan:
            if action[0] == 'pick':
                print(action[1][1])
                block_sequence.append(action[1][1])

        supp_res = planner.unpack_stability_results(goal_linked_state)
        supp_results[h.name] = supp_res
        block_sequences.append(block_sequence)

        if animate:
            animator = TrimeshBlockAnimator(init_config, goal_config, block_sequence, colors)
            animator.run()
        # animator.save_to_video(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}.mp4')
        # construction_animation = animate_construction_sequence(init_config, goal_config, block_sequence, colors, interval=500)
        # construction_animation.save(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}_full.gif', writer='pillow')
    else:
        print(f"Goal linked state not found :(.")

    if show:
        # Visualize stability over each step in the plan
        # Visualize total support scores for each planner
        # Visualize average support scores for each planner
        fig, axs = plt.subplots(2, 1, figsize=(12, 5))
        steps = np.arange(0, len(supp_results[h.name][0]), 1, dtype=int)
        colors = ['b', 'r']
        for i, (h_name, res) in enumerate(supp_results.items()):
            axs[i].plot(steps, res[0], label=h_name, color=colors[i])  # Stability score at each step
            axs[i].scatter(steps, res[0], s=100, c=colors[i])  # Highlight block placements
            print(f"{h_name} - Average Support Score: {res[2]}, Total Support Score: {res[1]}, all scores: {res[0]}")
        axs[0].set_title("Stability Score at Each Step")
        axs[0].set_xlabel("Step")
        axs[0].set_ylabel("Stability Score")
        axs[0].legend()
        axs[1].legend()

        fig, axs = plt.subplots(1, 1, figsize=(12, 5))
        for h_name, res in supp_results.items():
            axs.bar(h_name, res[1], label=h_name)
        axs.set_title("Total Support Score for Final Structure")
        axs.set_ylabel("Total Support Score")
        axs.legend()

        fig, axs = plt.subplots(1, 1, figsize=(12, 5))
        for h_name, res in supp_results.items():
            axs.bar(h_name, res[2], label=h_name)
        axs.set_title("Average Support Score for Final Structure")
        axs.set_ylabel("Average Support Score")
        axs.legend()
        plt.show()

if __name__ == "__main__":
    main()