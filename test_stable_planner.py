import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd

from copy import deepcopy
from yaml import safe_load

from modular_construction_task_planner.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC
from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.block_domain import PickAction, PlaceAction, TransitAction, TransportAction
from modular_construction_task_planner.stability import (
    visualize_goal_structure,
    create_support_relation_graph,
    visualize_support_node_graph,
    animate_construction_sequence,
    find_feasible_block_sequence,
    generate_nice_colors,
    save_construction_sequence_frames
)
from modular_construction_task_planner.block_sequence_animator import TrimeshBlockAnimator
from modular_construction_task_planner.astar import GridGraph, OCCUPANCY

def main(problem_name:str = "scaffolding_tower", show: bool = False, animate: bool = False, h: HEURISTIC = HEURISTIC.STABLE):
    goal_linked_state = None
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    world = parse_configs_to_world(problem_name, problem_config_path)
    # for ent in world.entities.entities:
        # print(f"{ent.name}: {ent.state}")

    init_config = safe_load(open(f"{problem_config_path}/{problem_name}/init.yaml", 'r'))
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    ground_mesh, support_graph = create_support_relation_graph(world)
    seq = find_feasible_block_sequence(support_graph)

    colors = generate_nice_colors(len(goal_config))

    if show:
        visualize_goal_structure(goal_config, title=problem_name, show=show, unique_colors=colors)
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
    cost = np.nan
    block_sequences = []
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
            # animator = TrimeshBlockAnimator(init_config, goal_config, block_sequence, colors)
            # animator.run()
            # animator.save_to_video(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}.mp4')
            # construction_animation = animate_construction_sequence(init_config, goal_config, block_sequence, colors, interval=500, title=problem_name)
            # construction_animation.save(f'src/modular_contruction_task_planner/modular_construction_task_planner/modular_construction_task_planner/movies/{problem_name}_full.gif', writer='pillow')
            save_construction_sequence_frames(init_config, goal_config, block_sequence, colors, title=problem_name)
    else:
        print(f"Goal linked state not found :(.")

    return (*supp_res, lazy_nav_cost)

if __name__ == "__main__":
    supp_results_lazy = {}
    supp_results_stab = {}
    supp_results_stab_nav = {}
    # problems = [ "quadruple_towers", "interlocking_pyramid", "scaffolding_tower"]
    problems = ["temple_facade"]
    colors = ['b', 'r', 'g']
    heuristics = [HEURISTIC.LAZY, HEURISTIC.STABLE, HEURISTIC.STABLE_NAV]

    sns.set_theme(style="whitegrid")
    plt.rcParams.update({'font.size': 16, 'axes.labelsize': 16, 'axes.titlesize': 16, 'xtick.labelsize': 12, 'ytick.labelsize': 12})
    figsize = (6, 4)

    h = HEURISTIC.LAZY
    for problem in problems:
        print(f"Testing problem: {problem}")
        supp_results = main(problem_name=problem, h=h, animate=True)
        supp_results_lazy[problem] = supp_results

    h = HEURISTIC.STABLE
    for problem in problems:
        print(f"Testing problem: {problem}")
        supp_results = main(problem_name=problem, h=h)
        supp_results_stab[problem] = supp_results

    h = HEURISTIC.STABLE_NAV
    for problem in problems:
        print(f"Testing problem: {problem}")
        supp_results = main(problem_name=problem, h=h)
        supp_results_stab_nav[problem] = supp_results

    fig, ax = plt.subplots(len(supp_results_lazy), 1, figsize=figsize)
    for i, (problem, res) in enumerate(supp_results_lazy.items()):
        print(f"Results for {problem} with LAZY heuristic:")
        stab_per_step = res[0]
        steps = np.arange(0, len(stab_per_step), 1, dtype=int)
        ax[i].plot(steps, stab_per_step, label=problem, color=colors[i])  # Stability score at each step
        ax[i].scatter(steps, stab_per_step, s=100, c=colors[i])  # Highlight block placements
        ax[1].set_ylabel("Stability Score")
        ax[i].legend()
        if i < len(supp_results_lazy) - 1:
            ax[i].set_xticklabels([])  # Hide x-axis labels for all but the last subplot
        print(f"{problem} - Average Support Score: {res[2]}, Total Support Score: {res[1]}, nav cost: {res[3]}, all scores: {res[0]}")
    ax[0].set_title(f"Stability Score at Each Step")
    ax[i].set_xlabel("Step")

    # fig, ax = plt.subplots(len(supp_results_stab), 1, figsize=(6, 3 * len(supp_results_stab)))
    # for i, (problem, res) in enumerate(supp_results_stab.items()):
    #     print(f"Results for {problem} with STABLE heuristic:")
    #     stab_per_step = res[0]
    #     steps = np.arange(0, len(stab_per_step), 1, dtype=int)
    #     ax[i].plot(steps, stab_per_step, label=problem, color=colors[i])  # Stability score at each step
    #     ax[i].scatter(steps, stab_per_step, s=100, c=colors[i])  # Highlight block placements
    #     ax[1].set_ylabel("Stability Score")
    #     ax[i].legend()
    #     print(f"{problem} - Average Support Score: {res[2]}, Total Support Score: {res[1]}, nav cost: {res[3]}, all scores: {res[0]}")
    # ax[0].set_title(f"Stability Score at Each Step")
    # ax[i].set_xlabel("Step")

    # fig, ax = plt.subplots(len(supp_results_stab_nav), 1, figsize=(6, 3 * len(supp_results_stab_nav)))
    # for i, (problem, res) in enumerate(supp_results_stab_nav.items()):
    #     print(f"Results for {problem} using STABLE_NAV heuristic")
    #     stab_per_step = res[0]
    #     steps = np.arange(0, len(stab_per_step), 1, dtype=int)
    #     ax[i].plot(steps, stab_per_step, label=problem, color=colors[i])  # Stability score at each step
    #     ax[i].scatter(steps, stab_per_step, s=100, c=colors[i])  # Highlight block placements
    #     ax[1].set_ylabel("Stability Score")
    #     ax[i].legend()
    #     print(f"{problem} - Average Support Score: {res[2]}, Total Support Score: {res[1]}, nav cost: {res[3]}, all scores: {res[0]}")
    # ax[0].set_title(f"Stability Score at Each Step")
    # ax[i].set_xlabel("Step")

    nav_cost_per_heuristic = {
        HEURISTIC.LAZY.name: [res[3] for res in supp_results_lazy.values()],
        HEURISTIC.STABLE.name: [res[3] for res in supp_results_stab.values()],
        HEURISTIC.STABLE_NAV.name: [res[3] for res in supp_results_stab_nav.values()]
    }

    nav_df_col = ['problem', 'heuristic', 'nav_cost']
    nav_df = pd.DataFrame(columns=nav_df_col)
    idx = 0
    for heuristic_name, nav_costs in nav_cost_per_heuristic.items():
        for problem, nav_cost in zip(problems, nav_costs):
            nav_df.loc[idx] = [problem, heuristic_name, nav_cost]
            idx += 1

    x = np.arange(len(problems))
    width = 0.25
    multiplier = 0
    fig, ax = plt.subplots(layout='constrained', figsize=(10, 6))

    # for heuristic_name, nav_costs in nav_cost_per_heuristic.items():
    #     offset = width * multiplier
    #     ax = sns.barplot(x=x + offset, y=nav_costs, width=width, label=heuristic_name, color=colors[multiplier])  # type: ignore
    #     # rects = ax.bar(x + offset, nav_costs, width, label=heuristic_name)
    #     # ax.bar_label(rects, padding=3)
    #     multiplier += 1

    ax = sns.barplot(data=nav_df, x='problem', y='nav_cost', hue='heuristic', palette=colors)

    # ax.set_xticks(x + width, problems)
    ax.set_ylabel("Navigation Cost [m]")
    ax.set_title("Navigation Cost by Heuristic per problem")
    ax.legend()
    plt.show()