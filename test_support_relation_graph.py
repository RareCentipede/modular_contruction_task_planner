import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon
from typing import Optional, Dict, Tuple, List
from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.stability import (
    create_support_relation_graph,
    visualize_goal_structure,
    visualize_support_node_graph,
    find_feasible_block_sequence,
    generate_nice_colors
)

def main(problem_config_path: str = "configs/problem_configs/",
         problem_name: str = "temple_facade"):
    # Load world from config
    world = parse_configs_to_world(problem_name, problem_config_path)

    plt.rcParams.update({'font.size': 20, 'axes.labelsize': 15, 'axes.titlesize': 20})

    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    colors = generate_nice_colors(len(goal_config))
    visualize_goal_structure(goal_config, show=False, unique_colors=colors, title=problem_name)

    plt.savefig(f"plots/struct_vis_{problem_name}.png")

    # Create support relation graph
    _, support_graph = create_support_relation_graph(world)
    visualize_support_node_graph(support_graph, colors=colors, show=False, title=f"Support Relation Graph - {problem_name}")

    feasible_seq = find_feasible_block_sequence(support_graph)
    print("Feasible block placement sequence (bottom to top):")
    for idx, block_name in enumerate(feasible_seq):
        print(f"{idx+1}. {block_name}")

    # Print support relationships
    for obj_name, node in support_graph.items():
        print(f"Object: {obj_name}")
        print(f"  Supported: {node.supported} (Score: {node.current_support_score:.2f}/{node.support_threshold})")
        print(f"  Supports: {[f'{name} (Score: {score:.2f})' for name, (score, _) in node.supported_objects.items()]}")
        print(f"  Supported by: {[f'{name} (Score: {score:.2f})' for name, (score, _) in node.supporting_objects.items()]}")
        print()

    # plt.show()
    plt.savefig(f"plots/support_relation_graph_{problem_name}.png")

if __name__ == "__main__":
    problems = [
        "temple_facade",
        "scaffolding_tower",
        "quadruple_towers",
        "interlocking_pyramid"
    ]
    for problem in problems:
        main(problem_name=problem)