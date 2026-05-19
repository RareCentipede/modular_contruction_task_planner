import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon
from typing import Optional, Dict, Tuple, List
from yaml import safe_load

from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.scripts.stability import (
    create_support_relation_graph,
    visualize_goal_structure,
    visualize_support_node_graph,
    find_feasible_block_sequence
)

def main():
    # Load world from config
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    problem_name = "temple_facade"
    world = parse_configs_to_world(problem_name, problem_config_path)

    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    colors = visualize_goal_structure(goal_config, show=False)

    # Create support relation graph
    _, support_graph = create_support_relation_graph(world)
    visualize_support_node_graph(support_graph, colors=colors, show=False)

    feasible_seq = find_feasible_block_sequence(support_graph)
    print("Feasible block placement sequence (bottom to top):")
    for idx, block_name in enumerate(feasible_seq):
        print(f"{idx+1}. {block_name}")

    # Print support relationships
    for obj_name, node in support_graph.items():
        print(f"Object: {obj_name}")
        print(f"  Supported: {node.supported} (Score: {node.current_support_score:.2f}/{node.support_threshold})")
        print(f"  Supports: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supported_objects]}")
        print(f"  Supported by: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supporting_objects]}")
        print()

    plt.show()

if __name__ == "__main__":
    main()