from typing import Dict, List, cast

from geometry_msgs.msg import PoseStamped
from modular_construction_task_planner.eas.core import (
    load_domains,
    Entities,
    World,
    Pose
)
from modular_construction_task_planner.scripts.block_domain import (
    Object,
    Surface,
    PosEntity,
    Robot
)
from modular_construction_task_planner.eas.config_parser_world_basic import create_entities, define_goal_state
from mpnp_interfaces.msg import Block

def parse_block_list_to_world(block_list: List[Block], robot_init_pose: PoseStamped, verbose: bool = False) -> World:
    block_var_domain = []
    robo_pos_var_domain = []
    pos_var_domain = []
    bool_var_domain = (True, False)

    entities = []
    pose_dict = {}

    robo_pos_var_domain.append(robot_init_pose.header.frame_id)
    pos_var_domain.append(robot_init_pose.header.frame_id)

    pose_dict[robot_init_pose.header.frame_id] = Pose(
        position=[robot_init_pose.pose.position.x, robot_init_pose.pose.position.y, robot_init_pose.pose.position.z],
        orientation=[robot_init_pose.pose.orientation.x, robot_init_pose.pose.orientation.y,
                     robot_init_pose.pose.orientation.z, robot_init_pose.pose.orientation.w]
    )

    define_poses_in_domain(block_list, pose_dict, block_var_domain, pos_var_domain, robo_pos_var_domain)

    pos_var_domain.append('g')
    pos_var_domain.append('')
    block_var_domain.append('g')
    block_var_domain.append('')

    domains = {
        'pos': tuple(pos_var_domain),
        'robo_pos': tuple(robo_pos_var_domain),
        'block': tuple(block_var_domain),
        'bool': bool_var_domain
    }
    load_domains(domains)
    entities = create_entities(domains)

    define_base_and_place_poses(block_list, entities)

    robot_entity = cast(Robot, entities.get_entities('robot'))
    robot_entity.at.value = robot_init_pose.header.frame_id

    if verbose:
        print(f"Domains:\n{domains}\n")

        for ent in entities.entities:
            print(ent.state)

    goal_state = define_goal_state(entities)
    world = World(entities, pose_dict, goal_state=goal_state)

    return world

def define_poses_in_domain(block_list: List[Block], pose_dict: Dict[str, Pose],
                           block_var_domain: List[str], pos_var_domain: List[str], robo_pos_var_domain: List[str]):
    for block in block_list:
        block_var_domain.append(block.name)
        pos_var_domain.append(block.init_pose.header.frame_id)
        pos_var_domain.append(block.goal_pose.header.frame_id)

        init_pose = Pose(
            position=[block.init_pose.pose.position.x, block.init_pose.pose.position.y, block.init_pose.pose.position.z],
            orientation=[block.init_pose.pose.orientation.x, block.init_pose.pose.orientation.y,
                         block.init_pose.pose.orientation.z, block.init_pose.pose.orientation.w]
        )
        pose_dict[block.init_pose.header.frame_id] = init_pose

        if block.goal_pose.header.frame_id != '':
            goal_pose = Pose(
                position=[block.goal_pose.pose.position.x, block.goal_pose.pose.position.y, block.goal_pose.pose.position.z],
                orientation=[block.goal_pose.pose.orientation.x, block.goal_pose.pose.orientation.y,
                             block.goal_pose.pose.orientation.z, block.goal_pose.pose.orientation.w]
            )
            pose_dict[block.goal_pose.header.frame_id] = goal_pose

        for base_pos, place_pos in zip(block.base_positions, block.place_positions):
            base_pos_name = base_pos.header.frame_id
            robo_pos_var_domain.append(base_pos_name)
            place_pos_name = place_pos.header.frame_id
            robo_pos_var_domain.append(place_pos_name)

            base_pose = Pose(
                position=[base_pos.pose.position.x, base_pos.pose.position.y, base_pos.pose.position.z],
                orientation=[base_pos.pose.orientation.x, base_pos.pose.orientation.y,
                             base_pos.pose.orientation.z, base_pos.pose.orientation.w]
            )

            place_pose = Pose(
                position=[place_pos.pose.position.x, place_pos.pose.position.y, place_pos.pose.position.z],
                orientation=[place_pos.pose.orientation.x, place_pos.pose.orientation.y,
                             place_pos.pose.orientation.z, place_pos.pose.orientation.w]
            )

            pose_dict[base_pos_name] = base_pose
            pose_dict[place_pos_name] = place_pose

def define_base_and_place_poses(block_list: List[Block], entities: Entities):
    for block in block_list:
        surfaces = []
        block_name = block.name

        init_pose_name = block.init_pose.header.frame_id
        init_pos_entity = cast(PosEntity, entities.get_entities(init_pose_name))
        init_pos_entity.on.value = cast(PosEntity, entities.get_entities('g')).name
        init_pos_entity.clear.value = False
        init_pos_entity.occupied_by.value = block_name

        block_entity = cast(Object, entities.get_entities(block_name))
        block_entity.goal.value = block.goal_pose.header.frame_id if not block.goal_pose.header.frame_id == '' else None
        block_entity.on.value = cast(PosEntity, entities.get_entities('g')).name
        block_entity.at.value = init_pose_name

        for surface_msg in block.surfaces:
            surface = Surface(
                vertices=[[vert.x, vert.y, vert.z] for vert in surface_msg.vertices],
                center=[surface_msg.center.x, surface_msg.center.y, surface_msg.center.z],
                normal=[surface_msg.normal.x, surface_msg.normal.y, surface_msg.normal.z]
            )
            surfaces.append(surface)
        block_entity.surfaces = surfaces

        bps = [base_pos.header.frame_id for base_pos in block.base_positions]
        pps = [place_pos.header.frame_id for place_pos in block.place_positions]
        block_entity.reachable_from = bps
        block_entity.placeable_from = pps
