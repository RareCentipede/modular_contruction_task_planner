import numpy as np

from shapely.geometry import Polygon
from typing import Optional, Dict, Tuple, List
from yaml import safe_load

from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.scripts.stability import (
    create_support_relation_graph,
    visualize_goal_structure,
    visualize_support_node_graph
)

def main():
    # Load world from config
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    problem_name = "temple_facade"
    world = parse_configs_to_world(problem_name, problem_config_path)

    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))
    visualize_goal_structure(goal_config)

    # Create support relation graph
    support_graph = create_support_relation_graph(world)
    visualize_support_node_graph(support_graph)

    # Print support relationships
    for obj_name, node in support_graph.items():
        print(f"Object: {obj_name}")
        print(f"  Supported: {node.supported} (Score: {node.current_support_score:.2f}/{node.support_threshold})")
        print(f"  Supports: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supported_objects]}")
        print(f"  Supported by: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supporting_objects]}")
        print()

if __name__ == "__main__":
    main()