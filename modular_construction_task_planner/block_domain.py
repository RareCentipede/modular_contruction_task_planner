import numpy as np

from trimesh import Trimesh, creation
from typing import Dict, List, Tuple, Callable, cast
from dataclasses import dataclass, field
from eas.core import (
    Entities, Variable, Entity, State, Condition, ComputedCondition, Effect, Action, Pose, World
)
# Add this import to block_domain.py
from modular_construction_task_planner.rbe_solver import compute_stablelego_equilibrium

world = World(Entities([]))  # Initialize an empty world
def load_world(new_world: World):
    global world
    world = new_world

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

@dataclass
class Host(Variable):
    domain: str = 'block'

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
class Surface:
    vertices: List[List[float]] # List of vertices that define the surface, a vertex is a list of 3 floats [x, y, z]
    center: List[float]
    normal: List[float]

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
    surfaces: List[Surface] = field(default_factory=list)
    mass: float = 1.0
    dim: List[float] = field(default_factory=list) # [length, width, height]
    _mesh: Trimesh | None = None

    @property
    def mesh(self) -> Trimesh:
        if self._mesh:
            return self._mesh

        if not self.surfaces:
            mesh = creation.box(extents=self.dim)
            self._mesh = mesh
            return mesh

        all_verts, all_faces = [], []
        offset = 0

        for surface in self.surfaces:
            verts = np.array(surface.vertices)
            # Triangulate the face (assumes convex polygon)
            faces = [[0, i, i+1] for i in range(1, len(verts)-1)]
            all_verts.append(verts)
            all_faces.extend([[f[0]+offset, f[1]+offset, f[2]+offset]
                            for f in faces])
            offset += len(verts)

        mesh = Trimesh(
            vertices=np.vstack(all_verts),
            faces=np.array(all_faces)
        )
        self._mesh = mesh
        return mesh

    def __post_init__(self):
        self.at_top.value = True
        self.supported.value = True

@dataclass
class ShadowBox(Entity):
    host: Host = field(default_factory=Host)
    at: At = field(default_factory=At)

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

def robot_can_reach_object_grasp(robot: Robot, obj: Object) -> bool:
    return robot.at.value in obj.reachable_from

def robot_can_reach_object_place(robot: Robot, obj: Object) -> bool:
    return robot.at.value in obj.placeable_from

def check_placement_stability(obj: Object,
                              target_pose: PosEntity) -> bool:
    """
    ComputedCondition computer function: Temporarily assigns the target pose to the object 
    and checks if the entire resulting structure remains in static equilibrium.
    """
    # Store original position
    orig_at = obj.at.value

    # Temporarily set object location
    obj.at.value = target_pose.name

    # Evaluate global multi-body equilibrium
    world_objects = cast(List[Object], world.entities.get_entities(Object))
    world_poses = world.pose_dict
    is_stable, _, _ = compute_stablelego_equilibrium(world_objects, world_poses)

    # Revert temporary assignment
    obj.at.value = orig_at

    return is_stable

# Action definitions
transit_parameters = {
    'robot': Robot,
    'start_pose': PosEntity,
    'target_pose': PosEntity,
    'object': Object
}
transit_conditions = [
    Condition('robot_at_start', 'robot', 'at', 'start_pose'),
]
transit_effects = [
    Effect('move_robot_to_target', 'robot', 'at', 'target_pose'),
    Effect('start_pose_clear', 'start_pose', 'clear', True),
    Effect('target_pose_not_clear', 'target_pose', 'clear', False),
]
TransitAction = Action('transit', transit_parameters, transit_conditions, transit_effects)

pick_parameters = {
    'robot': Robot,
    'object': Object,
    'target_pose': PosEntity
}
pick_conditions = [
    ComputedCondition('robot_can_reach_object', robot_can_reach_object_grasp, ('robot', 'object')),
    Condition('object_at_pose', 'object', 'at', 'target_pose'),
    Condition('gripper_empty', 'robot', 'gripper_empty', True),
    Condition('holding_nothing', 'robot', 'holding', None),
    Condition('object_at_top', 'object', 'at_top', True)
]
pick_effects = [
    Effect('pick_object', 'robot', 'holding', 'object'),
    Effect('gripper_not_empty', 'robot', 'gripper_empty', False),
    Effect('object_no_longer_at_pose', 'object', 'at', None),
    Effect('target_pose_clear', 'target_pose', 'clear', True),
    Effect('target_pose_occupied_by_none', 'target_pose', 'occupied_by', None),
    # Effect('object_on_none', 'object', 'on', None),
]
PickAction = Action('pick', pick_parameters, pick_conditions, pick_effects)

transport_parameters = {
    'robot': Robot,
    'start_pose': PosEntity,
    'target_pose': PosEntity,
    'object': Object
}
transport_conditions = [
    Condition('robot_at_start', 'robot', 'at', 'start_pose'),
    Condition('gripper_not_empty', 'robot', 'gripper_empty', False),
    Condition('holding_object', 'robot', 'holding', 'object'),
    Condition('target_pos_clear', 'target_pose', 'clear', True)
]
transport_effects = [
    Effect('move_robot_to_goal', 'robot', 'at', 'target_pose'),
    Effect('start_pose_clear', 'start_pose', 'clear', True),
    Effect('target_pose_not_clear', 'target_pose', 'clear', False),
]
TransportAction = Action('transport', transport_parameters, transport_conditions, transport_effects)

place_parameters = {
    'robot': Robot,
    'object': Object,
    'target_pose': PosEntity,
}
place_conditions = [
    ComputedCondition('robot_can_reach_object', robot_can_reach_object_place, ('robot', 'object')),
    Condition('gripper_holding_object', 'robot', 'holding', 'object'),
    Condition('target_pose_clear', 'target_pose', 'clear', True),
    Condition('object_supported', 'object', 'supported', True),
    # Integrated StableLego Force Equilibrium Precondition
    ComputedCondition('stack_statically_stable', check_placement_stability, ('object', 'target_pose'))
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