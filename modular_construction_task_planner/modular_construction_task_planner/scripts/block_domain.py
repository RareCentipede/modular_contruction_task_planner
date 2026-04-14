from typing import List, Tuple
from dataclasses import dataclass, field
from modular_construction_task_planner.eas.core import Variable, Entity, State

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
class Object(Entity):
    at: At = field(default_factory=At)
    at_top: AtTop = field(default_factory=AtTop)
    on: OnBlock = field(default_factory=OnBlock)
    below: BelowBlock = field(default_factory=BelowBlock)
    supported: Supported = field(default_factory=Supported)
    goal: Goal = field(default_factory=Goal)

    def __post_init__(self):
        self.at_top.value = True
        self.supported.value = True

@dataclass
class Pose(Entity):
    occupied_by: OccupiedBy = field(default_factory=OccupiedBy)
    clear: Clear = field(default_factory=Clear)
    on: OnPose = field(default_factory=OnPose)
    below: BelowPose = field(default_factory=BelowPose)

@dataclass
class Robot(Entity):
    at: At = field(default_factory=At)
    gripper_empty: GripperEmpty = field(default_factory=GripperEmpty)
    holding: Holding = field(default_factory=Holding)