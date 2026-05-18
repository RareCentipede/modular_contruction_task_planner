import trimesh
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon
from typing import Optional, Dict, Tuple, List

from modular_construction_task_planner.eas.core import World
from modular_construction_task_planner.scripts.block_domain import Object
from modular_construction_task_planner.eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.scripts.stability import SupportNode, create_support_relation_graph

def main():
    # Load world from config
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    world = parse_configs_to_world("quadriple_towers", problem_config_path)

    # Create support relation graph
    support_graph = create_support_relation_graph(world)

    # Print support relationships
    for obj_name, node in support_graph.items():
        print(f"Object: {obj_name}")
        print(f"  Supported: {node.supported} (Score: {node.current_support_score:.2f}/{node.support_threshold})")
        print(f"  Supports: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supported_objects]}")
        print(f"  Supported by: {[f'{name} (Score: {score:.2f})' for name, score, _ in node.supporting_objects]}")
        print()

if __name__ == "__main__":
    main()