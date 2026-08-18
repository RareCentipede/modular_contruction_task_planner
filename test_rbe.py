import numpy as np
from eas.core import World, Entities, Pose, load_domains
from modular_construction_task_planner.block_domain import Object, PosEntity, Robot
from modular_construction_task_planner.stability import (
    create_support_relation_graph,
    find_feasible_block_sequence,
    visualize_support_node_graph,
    visualize_goal_structure,
    animate_construction_sequence,
    generate_nice_colors
)

def run_test():
    # 1. Register finite variable domains required by core.py
    load_domains({
        'pos': (None, 'pose_block1_init', 'pose_block2_init', 'pose_block3_init', 
                'pose_block1_goal', 'pose_block2_goal', 'pose_block3_goal'),
        'bool': (True, False),
        'block': (None, 'block1', 'block2', 'block3'),
        'robo_pos': ('robot_home',)
    })

    # 2. Define block dimensions [length, width, height]
    dim = [1.0, 1.0, 1.0]

    # 3. Create block entities and assign goal pose references
    b1 = Object(name='block1', dim=dim)
    b2 = Object(name='block2', dim=dim)
    b3 = Object(name='block3', dim=dim)

    b1.goal.value = 'pose_block1_goal'
    b2.goal.value = 'pose_block2_goal'
    b3.goal.value = 'pose_block3_goal'

    entities = Entities([b1, b2, b3])

    # 4. Map poses (positions in world space)
    # Staging/Initial positions (Z = 0.5 for center of 1x1x1 cube)
    # Structure setup: block1 rests on ground, block2 on block1, block3 on block2
    pose_dict = {
        'pose_block1_init': Pose(position=[-2.0, 0.0, 0.5], orientation=[0, 0, 0]),
        'pose_block2_init': Pose(position=[-2.0, 2.0, 0.5], orientation=[0, 0, 0]),
        'pose_block3_init': Pose(position=[-2.0, -2.0, 0.5], orientation=[0, 0, 0]),

        'pose_block1_goal': Pose(position=[0.0, 0.0, 0.5], orientation=[0, 0, 0]),
        'pose_block2_goal': Pose(position=[0.0, 0.0, 1.5], orientation=[0, 0, 0]),
        'pose_block3_goal': Pose(position=[0.0, 0.0, 2.5], orientation=[0, 0, 0]),
    }

    world = World(entities=entities, pose_dict=pose_dict)

    # 5. Calculate support graph using stability.py
    print("Computing support relation graph...")
    ground_mesh, support_graph = create_support_relation_graph(
        world=world, 
        support_ratio_threshold=0.7, 
        verbose=True
    )

    # 6. Topological sort to determine placement order
    placement_sequence = find_feasible_block_sequence(support_graph)
    print(f"\nFeasible Placement Sequence: {placement_sequence}")

    # 7. Format dataset dicts for visualization functions
    init_data = {}
    goal_data = {}
    for obj in [b1, b2, b3]:
        init_p = world.pose_dict[f"pose_{obj.name}_init"].position
        goal_p = world.pose_dict[obj.goal.value].position # type: ignore

        init_data[obj.name] = {'position': init_p, 'size': obj.dim}
        goal_data[obj.name] = {'position': goal_p, 'size': obj.dim}

    colors = generate_nice_colors(len(placement_sequence))

    # 8. Display visual graph of support dependencies
    print("\nDisplaying support graph...")
    visualize_support_node_graph(
        support_graph=support_graph,
        colors=colors,
        title="Block Support Dependency Graph",
        show=True
    )

    # 9. Visualize goal structure in 3D
    print("Displaying goal structure...")
    visualize_goal_structure(
        goal_data=goal_data,
        unique_colors=colors,
        title="Goal Assembly Structure",
        show=True
    )

    # 10. Animate assembly sequence
    print("Animating sequence...")
    anim = animate_construction_sequence(
        init_data=init_data,
        goal_data=goal_data,
        placement_sequence=placement_sequence,
        color_array=colors,
        interval=1000,
        show=True,
        title="StableLego_Assembly_Test"
    )

if __name__ == "__main__":
    run_test()