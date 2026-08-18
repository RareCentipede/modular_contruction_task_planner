from yaml import safe_load

from eas.config_parser_world_basic import parse_configs_to_world
from modular_construction_task_planner.stability import (
    create_support_relation_graph,
    find_feasible_block_sequence,
    visualize_support_node_graph,
    visualize_goal_structure,
    animate_construction_sequence,
    generate_nice_colors
)


def run_test(problem_name: str = "scaffolding_tower", problem_config_path: str = "configs/problem_configs/"):
    # 1. Parse world definition and object states from config files
    world = parse_configs_to_world(problem_name, problem_config_path)

    # 2. Load raw YAML configurations for initial and goal layouts
    init_config = safe_load(open(f"{problem_config_path}/{problem_name}/init.yaml", 'r'))
    goal_config = safe_load(open(f"{problem_config_path}/{problem_name}/goal.yaml", 'r'))

    # 3. Compute support relation graph using the world instance
    print(f"Computing support graph for problem: {problem_name}...")
    ground_mesh, support_graph = create_support_relation_graph(
        world=world, 
        support_ratio_threshold=0.7, 
        verbose=True
    )

    # 4. Extract feasible placement sequence
    sequence = find_feasible_block_sequence(support_graph)
    if not sequence:
        print("No valid block placement sequence found due to unstable support constraints.")
        return

    print(f"\nFeasible Placement Sequence: {sequence}")

    # 5. Generate matching color palette based on block count
    colors = generate_nice_colors(len(goal_config))

    # 6. Display visual graph of support dependencies
    print("\nDisplaying support graph...")
    visualize_support_node_graph(
        support_graph=support_graph,
        colors=colors,
        title=f"{problem_name} - Support Dependency Graph",
        show=True
    )

    # 7. Visualize target goal structure in 3D
    print("Displaying goal structure...")
    visualize_goal_structure(
        goal_data=goal_config,
        unique_colors=colors,
        title=f"{problem_name} - Goal Structure",
        show=True
    )

    # 8. Animate construction sequence using parsed configs and sequence
    print("Animating assembly sequence...")
    anim = animate_construction_sequence(
        init_data=init_config,
        goal_data=goal_config,
        placement_sequence=sequence,
        color_array=colors,
        interval=800,
        show=True,
        title=problem_name
    )

if __name__ == "__main__":
    # Test across parameterized problem configurations
    problems = ["scaffolding_tower"]
    for problem in problems:
        run_test(problem_name=problem)