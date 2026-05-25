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

def main(problem_name:str = "scaffolding_tower", show: bool = False, animate: bool = False, h: HEURISTIC = HEURISTIC.STABLE):
    goal_linked_state = None
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    world = parse_configs_to_world(problem_name, problem_config_path)
    for ent in world.entities.entities:
        print(f"{ent.name}: {ent.state}")

    init_config = safe_load(open(f"{problem_config_path}/{problem_name}/init.yaml", 'r'))
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    ground_mesh, support_graph = create_support_relation_graph(world)
    seq = find_feasible_block_sequence(support_graph)

    if show:
        colors = visualize_goal_structure(goal_config, title=problem_name, show=show)
        visualize_support_node_graph(support_graph, colors=colors, show=show)

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
    supp_res = ([], 0, 0)  # Initialize with empty scores
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
            # animator.save_to_video(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}.mp4')
            # construction_animation = animate_construction_sequence(init_config, goal_config, block_sequence, colors, interval=500)
            # construction_animation.save(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}_full.gif', writer='pillow')
    else:
        print(f"Goal linked state not found :(.")

    return supp_res

if __name__ == "__main__":
    all_supp_results = {}
    h = HEURISTIC.STABLE
    problems = [ "quadriple_towers", "interlocking_pyramid", "scaffolding_tower" ]
    colors = ['b', 'r', 'g']

    for problem in problems:
        print(f"Testing problem: {problem}")
        supp_results = main(problem_name=problem, show=True)
        all_supp_results[problem] = supp_results

    fig, ax = plt.subplots(len(all_supp_results), 1, figsize=(6, 3 * len(all_supp_results)))
    for i, (problem, res) in enumerate(all_supp_results.items()):
        print(f"Results for {problem}")
        stab_per_step = res[0]
        steps = np.arange(0, len(stab_per_step), 1, dtype=int)
        ax[i].plot(steps, stab_per_step, label=problem, color=colors[i])  # Stability score at each step
        ax[i].scatter(steps, stab_per_step, s=100, c=colors[i])  # Highlight block placements
        ax[i].set_ylabel("Stability Score")
        ax[i].legend()
        print(f"{problem} - Average Support Score: {res[2]}, Total Support Score: {res[1]}, all scores: {res[0]}")
    ax[0].set_title(f"Stability Score at Each Step")
    ax[i].set_xlabel("Step")
    plt.show()