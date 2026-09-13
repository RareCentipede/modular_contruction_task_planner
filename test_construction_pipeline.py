import yaml
import os
import sys
import time
from typing import Dict, List, Optional, cast
import numpy as np
import matplotlib.pyplot as plt

# Core import dependencies
from eas.config_parser_world_basic import parse_configs_to_world
from eas.core import LinkedState, World
from modular_construction_task_planner.block_domain import Object, PickAction, PlaceAction, TransitAction, TransportAction, load_world
from modular_construction_task_planner.ordered_landmarks_planner import OrderedLandmarksPlanner, HEURISTIC
from modular_construction_task_planner.stability import (
    create_support_relation_graph,
    find_feasible_block_sequence,
    visualize_goal_structure,
    visualize_support_node_graph,
    generate_nice_colors
)
from modular_construction_task_planner.rbe_solver import compute_stablelego_equilibrium
from modular_construction_task_planner.block_sequence_animator import TrimeshBlockAnimator
from visualize_stability import compute_construction_metrics, plot_friction_heatmap, plot_construction_force_and_strain

action_dict = {
        'transit': TransitAction,
        'transport': TransportAction,
        'pick': PickAction,
        'place': PlaceAction
}

def retrace_placement_sequence_from_goal_linked_state(goal_linked_state) -> List[str]:
    """
    Retrace the placement sequence from the goal linked state.
    """
    placement_sequence = []
    current_state = goal_linked_state
    parent = current_state.parent
    while parent is not None:
        action = current_state.action_from_parent
        if action[0] == 'place':
            print(action)
            placement_sequence.append(action[1][1])
        current_state = parent[1]
        parent = current_state.parent

    placement_sequence.reverse()  # Reverse to get the correct order
    return placement_sequence

def validate_plan_stability(goal_linked_state, world: World) -> List[str]:
    """
    Retrace the placement sequence from the goal linked state.
    """
    stability_verdict = []
    current_state = goal_linked_state
    parent = current_state.parent
    objs = world.entities.get_entities(Object)
    objs = cast(List[Object], objs)
    obj_at_state_keys = [f"{obj.name}_at" for obj in objs]
    while parent is not None:
        action = current_state.action_from_parent
        if action[0] != 'place':
            current_state = parent[1]
            parent = current_state.parent
            continue

        if current_state.properties['verified']:
            stability_verdict.append('Pre-verified')
            current_state = parent[1]
            parent = current_state.parent
            continue

        obj_pos_in_state = [current_state.state[key] for key in obj_at_state_keys]
        for obj, pos in zip(objs, obj_pos_in_state):
            obj.at.value = pos
        is_stable, residuals, contact_forces = compute_stablelego_equilibrium(
            objects=objs,
            world_poses=world.pose_dict,
            default_mass=1.0,
            default_mu=0.5
        )
        if is_stable:
            stability_verdict.append('Stable')
        else:
            stability_verdict.append('Unstable')
            return stability_verdict  # Early exit on first unstable step
        current_state = parent[1]
        parent = current_state.parent
        stability_verdict.reverse()  # Reverse to get the correct order
    return stability_verdict

def run_construction_testing_pipeline(problem_name: str = "arch", 
                                      config_path: str = "configs/problem_configs/",
                                      init_config: Optional[Dict] = None,
                                      goal_config: Optional[Dict] = None,
                                      heuristic: HEURISTIC = HEURISTIC.AO,
                                      show: bool = False):
    print(f"==================================================")
    print(f" Running Construction Pipeline Test: [{problem_name}]")
    print(f"==================================================")

    # --------------------------------------------------------------------------
    # 1. World Initialization & Goal Parser
    # --------------------------------------------------------------------------
    if not init_config and not goal_config:
        print("\n1. Initializing World & Goal configurations...")
        world = parse_configs_to_world(problem_name, config_path)
        goal_file = os.path.join(config_path, problem_name, "goal.yaml")
        with open(goal_file, 'r') as f:
            goal_config = yaml.safe_load(f)
    else:
        print("\n1. Initializing World & Goal configurations from provided dictionaries...")
        world = parse_configs_to_world(init_config, goal_config) # type: ignore

    all_objects = world.entities.get_entities(Object)
    all_objects = cast(List[Object], all_objects)

    # Generate visually distinct colors for each block
    block_colors = generate_nice_colors(len(all_objects))
    print(f" Loaded {len(all_objects)} blocks successfully.")

    # --------------------------------------------------------------------------
    # 2. Structural Support Graph & Feasible Sequence Generation
    # --------------------------------------------------------------------------
    print("\n2. Computing Support Relation Graph...")
    ground_mesh, support_graph = create_support_relation_graph(world, support_ratio_threshold=0.7)

    try:
        placement_sequence = find_feasible_block_sequence(support_graph)
        print(f" Feasible Block Placement Sequence: {placement_sequence}")
    except ValueError as e:
        print(f" Error in sequence generation: {e}")
        return None, 0.0

    # --------------------------------------------------------------------------
    # 3. Symbolic Task Planning (OrderedLandmarksPlanner)
    # --------------------------------------------------------------------------
    print("\n3. Generating Plan using OrderedLandmarksPlanner...")

    # Instantiate and execute the OrderedLandmarksPlanner
    start_time = time.time()
    planner = OrderedLandmarksPlanner(world, action_dict)
    goal_linked_state = planner.run_stable_planner(support_graph, ground_mesh, h=heuristic)
    stability_verdict = []
    if heuristic == HEURISTIC.STABLE:
        stability_verdict = validate_plan_stability(goal_linked_state, world) if goal_linked_state else []
    print(f" Stability Verdict for each placement step: {stability_verdict}")
    end_time = time.time()
    time_taken = end_time - start_time
    print(f" Heuristic {heuristic.name} Planning completed in {time_taken:.2f} seconds.")

    if goal_linked_state:
        placement_sequence = retrace_placement_sequence_from_goal_linked_state(goal_linked_state)
        print(f" Landmark Plan Found! Steps: {len(placement_sequence)}")
        # Extract ordered sequence from the plan actions
    else:
        print(" Landmark Planner returned no path. Falling back to topological graph sorting...")
        ground_mesh, support_graph = create_support_relation_graph(world, support_ratio_threshold=0.5)
        placement_sequence = find_feasible_block_sequence(support_graph)

    print(f" Target Sequence: {placement_sequence}")

    # --------------------------------------------------------------------------
    # 4. Static Equilibrium Verification (rbe_solver)
    # --------------------------------------------------------------------------
    print("\n4. Testing Structural Equilibrium with rbe_solver...")

    # Temporarily assign goal poses to test full-assembly equilibrium
    for obj in all_objects:
        if obj.goal.value:
            obj.at.value = obj.goal.value

    is_stable, residuals, contact_forces = compute_stablelego_equilibrium(
        objects=all_objects,
        world_poses=world.pose_dict,
        default_mass=1.0,
        default_mu=0.5
    )

    print(f" Equilibrium Result: {'STABLE' if is_stable else 'UNSTABLE / INFEASIBLE'}")
    print(f" Planning results: {'Stable' if placement_sequence != [] else 'Unstable'}")

    # Reset entity states after full-structure check
    world.update_state()

    # --------------------------------------------------------------------------
    # 4. Visualization & Analysis Outputs
    # --------------------------------------------------------------------------
    print("\n4. Generating Structural Visualizations...")

    # A. 3D Goal Assembly Layout
    # visualize_goal_structure(goal_data, block_colors, title=f"Goal Structure - {problem_name}", show=False)

    # B. Topological Dependency Graph
    # visualize_support_node_graph(support_graph, colors=block_colors, title=f"Support Dependency Tree - {problem_name}", show=False)

    # C. Strain Heatmaps & Metrics
    # Full stable does not mean there is a feasible construction plan.
    if show:
        metrics_data = compute_construction_metrics(problem_name, world, goal_config, construction_sequence=placement_sequence) #type: ignore
        plot_friction_heatmap(metrics_data, show=False)
        plot_construction_force_and_strain(metrics_data, show=False)
        plt.show()
    # plt.pause(2.0)

    # # --------------------------------------------------------------------------
    # # 5. Interactive Assembly Animation (Trimesh & PyOpenGL)
    # # --------------------------------------------------------------------------
    # print("\n5. Launching Construction Sequence Animator...")
    # print("   Close the animation window when complete to finish execution.")

    # animator = TrimeshBlockAnimator(
    #     init_data=init_data,
    #     goal_data=goal_data,
    #     placement_sequence=placement_sequence,
    #     color_array=block_colors
    # )

    # # Run active 3D visualization window
    # animator.run()

    return isinstance(goal_linked_state, LinkedState), time_taken

if __name__ == "__main__":
    # Change "arch" to any existing configuration folder name in your system
    run_construction_testing_pipeline(problem_name="problem_01")