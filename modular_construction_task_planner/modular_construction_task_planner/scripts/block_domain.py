from typing import List, Tuple
from dataclasses import dataclass, field
from modular_construction_task_planner.eas.core import Variable, Entity, State, Condition, Effect, Action

# Block variables
@dataclass
class At(Variable):
    domain: str = 'pos'

@dataclass
class AtTop(Variable):
    domain: str = 'bool'

@dataclass
class OnBlock(Variable):
    domain: str = 'block'

@dataclass
class BelowBlock(Variable):
    domain: str = 'block'

@dataclass
class Supported(Variable):
    domain: str = 'bool'

@dataclass
class Goal(Variable):
    domain: str = 'pos'

# Pose variables
@dataclass
class Clear(Variable):
    domain: str = 'bool'

@dataclass
class OccupiedBy(Variable):
    domain: str = 'block'

@dataclass
class OnPose(Variable):
    domain: str = 'pos'

@dataclass
class BelowPose(Variable):
    domain: str = 'pos'

# Robot variables
@dataclass
class GripperEmpty(Variable):
    domain: str = 'bool'

@dataclass
class Holding(Variable):
    domain: str = 'block'

@dataclass
class AtGoal(Variable):
    domain: str = 'bool'

@dataclass
class RobotAt(Variable):
    domain: str = 'robo_pos'

@dataclass
class Object(Entity):
    at: At = field(default_factory=At)
    at_top: AtTop = field(default_factory=AtTop)
    on: OnBlock = field(default_factory=OnBlock)
    below: BelowBlock = field(default_factory=BelowBlock)
    supported: Supported = field(default_factory=Supported)
    goal: Goal = field(default_factory=Goal)
    reachable_from: List[str] = field(default_factory=list)
    placeable_from: List[str] = field(default_factory=list)

    def __post_init__(self):
        self.at_top.value = True
        self.supported.value = True

@dataclass
class PosEntity(Entity):
    occupied_by: OccupiedBy = field(default_factory=OccupiedBy)
    clear: Clear = field(default_factory=Clear)
    on: OnPose = field(default_factory=OnPose)
    below: BelowPose = field(default_factory=BelowPose)

    def __post_init__(self):
        self.clear.value = True

@dataclass
class Robot(Entity):
    at: RobotAt = field(default_factory=RobotAt)
    gripper_empty: GripperEmpty = field(default_factory=GripperEmpty)
    holding: Holding = field(default_factory=Holding)
    at_goal: AtGoal = field(default_factory=AtGoal)

    def __post_init__(self):
        self.gripper_empty.value = True
        self.holding.value = None
        self.at_goal.value = False

# Action definitions
transit_parameters = {
    'robot': Robot,
    'start_pose': PosEntity,
    'target_pose': PosEntity
}
transit_conditions = [
    Condition('robot_at_start', 'robot', 'at', 'start_pose'),
    Condition('robot_not_at_goal', 'robot', 'at_goal', False),
    Condition('target_pose_clear', 'target_pose', 'clear', True),
    # Condition('target_pose_reachable', 'target_object', 'reachable_from', 'target_pose') This needs to be different,
    # it should be a computed predicate since we need to check if target_pose is in target_object.reachable_from
]
transit_effects = [
    Effect('move_robot_to_target', 'robot', 'at', 'target_pose'),
    Effect('robot_not_at_start', 'start_pose', 'occupied_by', None),
    Effect('target_pose_occupied_by_robot', 'target_pose', 'occupied_by', 'robot'),
    Effect('target_pose_not_clear', 'target_pose', 'clear', False),
]

pick_parameters = {
    'robot': Robot,
    'object': Object,
    'object_pose': PosEntity
}
pick_conditions = [
    Condition('robot_at_object', 'robot', 'at', 'object_pose'),
    Condition('object_at_pose', 'object', 'at', 'object_pose'),
    Condition('gripper_empty', 'robot', 'gripper_empty', True),
    Condition('holding_nothing', 'robot', 'holding', None),
    Condition('object_at_top', 'object', 'at_top', True)
]
pick_effects = [
    Effect('pick_object', 'robot', 'holding', 'object'),
    Effect('gripper_not_empty', 'robot', 'gripper_empty', False),
    Effect('object_no_longer_at_pose', 'object', 'at', None),
    Effect('object_pose_clear', 'object_pose', 'clear', True),
    Effect('object_pose_occupied_by_none', 'object_pose', 'occupied_by', None),
    Effect('object_on_none', 'object', 'on', None),
]
PickAction = Action('pick', pick_parameters, pick_conditions, pick_effects)

transport_parameters = {
    'robot': Robot,
    'start_pos': PosEntity,
    'target_pose': PosEntity
}
transport_conditions = [
    Condition('robot_at_start', 'robot', 'at', 'start_pos'),
    Condition('robot_not_at_goal', 'robot', 'at_goal', False),
    Condition('gripper_not_empty', 'robot', 'gripper_empty', False),
    Condition('holding_object', 'robot', 'holding', 'object'),
    Condition('target_pos_clear', 'target_pose', 'clear', True)
]
transport_effects = [
    Effect('move_robot_to_goal', 'robot', 'at', 'target_pose'),
    Effect('target_pose_occupied_by_robot', 'target_pose', 'occupied_by', 'robot'),
    Effect('target_pose_not_clear', 'target_pose', 'clear', False),
    Effect('robot_not_at_start', 'start_pos', 'occupied_by', None),
    Effect('robot_at_goal', 'robot', 'at_goal', True),
]

place_parameters = {
    'robot': Robot,
    'object': Object,
    'target_pose': PosEntity
}
place_conditions = [
    Condition('robot_at_target', 'robot', 'at', 'target_pose'),
    Condition('gripper_holding_object', 'robot', 'holding', 'object'),
    Condition('target_pose_clear', 'target_pose', 'clear', True),
    Condition('object_supported', 'object', 'supported', True)
]
place_effects = [
    Effect('place_object', 'robot', 'holding', None),
    Effect('gripper_empty', 'robot', 'gripper_empty', True),
    Effect('object_at_target', 'object', 'at', 'target_pose'),
    Effect('target_pose_occupied_by_object', 'target_pose', 'occupied_by', 'object'),
    Effect('target_pose_not_clear', 'target_pose', 'clear', False),
    Effect('robot_not_at_goal', 'robot', 'at_goal', False),
]
PlaceAction = Action('place', place_parameters, place_conditions, place_effects)

move_parameters = {
    'robot': Robot,
    'start_pose': PosEntity,
    'target_pose': PosEntity
}
move_conditions = [
    Condition('robot_at_start', 'robot', 'at', 'start_pose')
]
move_effects = [
    Effect('move_robot_to_target', 'robot', 'at', 'target_pose')
]
MoveAction = Action('move', move_parameters, move_conditions, move_effects)