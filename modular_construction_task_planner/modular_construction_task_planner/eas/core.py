import numpy as np

from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, List, Type, Tuple, FrozenSet
from enum import Enum

# State is a snapshot of all variable values for a given state id. It is immutable once created,
# should be able to create new states from current entities' variables and set all variables based on a state snapshot.
# TODO: Make state into a FrozenSet type to ensure immutability and hashability for use in sets and dicts.
# TODO: I was too lazy to actually implement this lol, but it would be a great feature.
type State = Dict[str, Optional[Any]]
StateStatus = Enum('StateStatus', "ALIVE DEAD GOAL")

VarDomains: Dict[str, Tuple[Any, ...]] = {}

def load_domains(domains: Dict[str, Tuple[Any, ...]]) -> None:
    VarDomains.update(domains)

@dataclass
class Pose:
    position: List[float]
    orientation: List[float]

    @property
    def homogeneous(self) -> np.ndarray:
        T = np.eye(4)
        rotation = R.from_quat(self.orientation)
        T[:3, :3] = rotation.as_matrix()
        T[:3, 3] = self.position
        return T

    def transform_to_frame(self, from_pose: 'Pose', to_pose: 'Pose') -> np.ndarray:
        from_frame = from_pose.homogeneous
        to_frame = to_pose.homogeneous
        return np.linalg.inv(to_frame) @ from_frame @ self.homogeneous

@dataclass
class Variable:
    """
        A typed state variable with a finite domain.
        Holds the current value and validates on assignment.
    """
    domain: str
    _value: Optional[Any] = field(default=None, init=False, repr=False)

    @property
    def value(self) -> Optional[Any]:
        if not VarDomains:
            raise ValueError("VarDomains is not defined. Load domains using load_domains() before setting variable values.")
        return self._value

    @value.setter
    def value(self, v: Optional[Any]) -> None:
        if self.domain not in VarDomains:
            raise ValueError(f"Domain {self.domain!r} is not defined in VarDomains.")
        if v is not None and v not in VarDomains[self.domain]:
            raise ValueError(f"{v!r} not in domain {self.domain}: {VarDomains[self.domain]}")
        self._value = v

    def get_domain(self) -> Tuple[Any, ...]:
        if self.domain not in VarDomains:
            raise ValueError(f"Domain {self.domain!r} is not defined in VarDomains.")
        return VarDomains[self.domain]

    def __call__(self, index: Optional[int] = None) -> Optional[Any]:
        """
            Enumerate valid values by index.
        """
        if not index:
            return self._value

        return VarDomains[self.domain][index] if index < len(VarDomains[self.domain]) else None

    def __str__(self) -> Any:
        if not self._value:
            return f"∅ ∈ {VarDomains[self.domain]}"
        return f"{self._value!r} ∈ {VarDomains[self.domain]}"

@dataclass
class Entity:
    name: str

    @property
    def state(self) -> Dict[str, Any]:
        attrs = vars(self)

        state_dict = {}
        for attr_name, attr_value in attrs.items():
            if isinstance(attr_value, Variable):
                state_dict[f"{self.name}_{attr_name}"] = attr_value.value

        return state_dict

    def __str__(self) -> str:
        return f"{self.name}({', '.join(f'{k}={v}' for k, v in zip(list(vars(self).keys())[1:], self.state.values()))})"

@dataclass
class Entities:
    entities: List[Entity]
    _entities_by_type: Dict[Type[Entity], List[Entity]] = field(init=False, repr=False)
    _entities_by_name: Dict[str, Entity] = field(init=False, repr=False)

    def __post_init__(self):
        self._entities_by_type = {}
        self._entities_by_name = {}

        for ent in self.entities:
            ent_type = type(ent)
            if ent_type not in self._entities_by_type:
                self._entities_by_type[ent_type] = []
            self._entities_by_type[ent_type].append(ent)
            self._entities_by_name[ent.name] = ent

    def get_entities(self, key: str | Type[Entity]) -> Optional[Entity | List[Entity]]:
        if isinstance(key, str):
            return self._entities_by_name.get(key)
        elif isinstance(key, type) and issubclass(key, Entity):
            return self._entities_by_type.get(key)
        else:
            raise KeyError(f"Invalid key type: {key}. Must be str or Type[Entity].")

@dataclass
class Condition:
    name: str
    src_entity_name: str
    src_var_type: str
    target: Any
    negate: bool = False

    def __call__(self, src_entity: Entity, target_entity: Optional[Entity] = None, verbose: bool = False) -> bool:
        variables = vars(src_entity)
        var = variables.get(self.src_var_type)
        if var is None:
            raise ValueError(f"Condition {self.name} failed: {src_entity.name} has no variable of type {self.src_var_type}")

        target_val = target_entity.name if target_entity is not None else self.target
        if var.value != target_val:
            if verbose:
                print(f"Condition {self.name} failed: {src_entity.name}_{self.src_var_type}={var.value} != {target_val}")
            return False
        if verbose:
            print(f"Condition {self.name} passed: {src_entity.name}_{self.src_var_type}={var.value} == {target_val}")
        return True

@dataclass
class Effect:
    name: str
    src_entity_name: str
    src_var_type: str
    target: Any

    def __call__(self, src_entity: Entity, target_entity: Optional[Entity] = None, verbose: bool = False) -> bool:
        variables = vars(src_entity)
        var = variables.get(self.src_var_type)
        if var is None:
            if verbose:
                print(f"Effect {self.name} failed: {src_entity.name} has no variable of type {self.src_var_type}")
            return False
        try:
            var.value = target_entity.name if target_entity is not None else self.target
            if verbose:
                print(f"Effect {self.name} applied: {src_entity.name}_{self.src_var_type} set to {var.value}")
            return True
        except ValueError as e:
            raise RuntimeError(f"Effect {self.name} failed to apply: {e}")

@dataclass
class Action:
    name: str
    params: Dict[str, Type[Entity]]
    preconditions: List[Condition]
    effects: List[Effect]
    _checked: Optional[bool] = None

    def _type_check(self, entities: Dict[str, Entity]) -> bool:
        for ent_name, ent_type in self.params.items():
            if type(entities[ent_name]) != ent_type:
                raise TypeError(f"Parameter {ent_name} expected type {ent_type.__name__}, \
                                  got {type(entities[ent_name]).__name__}")
        return True

    def check(self, param_entities: Dict[str, Entity], verbose: bool = False) -> bool:
        self._type_check(param_entities)
        entities = {ent_name: param_entities[ent_name] for ent_name in self.params}

        for cond in self.preconditions:
            src_entity = entities[cond.src_entity_name]
            target = entities[cond.target] if isinstance(cond.target, str) else None

            try:
                if not cond(src_entity, target, verbose=verbose):
                    return False
            except ValueError as e:
                raise RuntimeError(f"Condition {cond.name} check failed with error: {e}")

        self._checked = True
        return True

    def execute(self, param_entities: Dict[str, Entity], verbose: bool = False) -> None:
        if self._checked is None:
            raise RuntimeError(f"Action {self.name} not checked before execution.")
        elif not self._checked:
            raise RuntimeError(f"Action {self.name} preconditions not satisfied. Cannot execute.")

        self._type_check(param_entities)
        entities = {ent_name: param_entities[ent_name] for ent_name in self.params}

        for eff in self.effects:
            src_entity = entities[eff.src_entity_name]
            target_entity = entities.get(eff.target) if isinstance(eff.target, str) else None
            try:
                eff(src_entity, target_entity, verbose=verbose)
            except ValueError as e:
                raise RuntimeError(f"Effect {eff.name} failed to apply: {e}")

    def __str__(self) -> str:
        return f"({self.name}])"

@dataclass
class LinkedState:
    state_id: int
    state: State
    parent: Optional[Tuple[str, 'LinkedState']] = None
    children: List[Tuple[str, 'LinkedState']] = field(default_factory=list) # List of expanded (action_name, LinkedState) pairs
    cost: float = 0.0
    goal: bool = False
    action_from_parent: Optional[Tuple[str, Tuple[str, ...]]] = None # (action_name, involved_entities) that led to this state from parent

    # List of potential branches: (action_name, action_params, cost)
    _branches_to_explore: List[Tuple[str, Dict[str, Entity], float]] = field(default_factory=list)
    _expanded: bool = False

    @property
    def branches_to_explore(self) -> List[Tuple[str, Dict[str, Entity], float]]:
        return self._branches_to_explore

    @branches_to_explore.setter
    def branches_to_explore(self, branches: List[Tuple[str, Dict[str, Entity], float]]) -> None:
        self._branches_to_explore = branches
        self._expanded = True

    @property
    def status(self) -> StateStatus:
        if not self._expanded:
            return StateStatus.ALIVE
        elif self.goal:
            return StateStatus.GOAL
        elif self.branches_to_explore:
            return StateStatus.ALIVE
        else:
            return StateStatus.DEAD

    def __hash__(self) -> FrozenSet:
        return frozenset(self.state.items())

    def __eq__(self, other: 'LinkedState') -> bool:
        return frozenset(self.state.items()) == frozenset(other.state.items())

    def __str__(self) -> str:
        return f"State {self.state_id} ({self.status.name}): {self.state}"

@dataclass
class World:
    # Maybe create another dataclass that can query for entities with multiple keys, like EntityType and EntityName
    # Entity name queries a single entity, while EntityType queries for all entities of that type
    entities: Entities
    pose_dict: Dict[str, Pose] = field(default_factory=dict)
    states: List[State] = field(default_factory=list)
    goal_state: State = field(default_factory=dict)

    def __post_init__(self):
        self.update_state()

    @property
    def current_state(self) -> State:
        return self.states[-1] if self.states else {}

    @property
    def goal_reached(self) -> bool:
        for state_key, state_val in self.goal_state.items():
            current_val = self.current_state.get(state_key)
            if current_val != state_val:
                return False

        return True

    @property
    def not_at_goal_entities(self) -> List[Entity]:
        not_at_goal_entities = []
        for state_key, state_val in self.goal_state.items():
            current_val = self.current_state.get(state_key)
            if current_val != state_val:
                ent_name = state_key.split("_")[0]
                entity = self.entities.get_entities(ent_name)
                not_at_goal_entities.append(entity)

        return not_at_goal_entities

    def update_state(self) -> State:
        new_state = deepcopy(self.current_state)

        for ent in self.entities.entities:
            new_state.update(ent.state)

        self.states.append(new_state)
        return new_state

    def update_entities_from_state(self, state: State) -> None:
        for ent in self.entities.entities:
            for var_name, var in vars(ent).items():
                var_key = f"{ent.name}_{var_name}"
                if var_key in state:
                    var_val = state[var_key]
                    try:
                        var.value = var_val
                        setattr(ent, var_name, var)
                    except ValueError as e:
                        print(f"Failed to update {ent.name}'s variable {var_name} with value {var_val}: {e}")