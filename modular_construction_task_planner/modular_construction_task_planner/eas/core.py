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

VarDomains: Dict[str, List[Any]] = {}

def load_domains(domains: Dict[str, List[Any]]) -> None:
    VarDomains.update(domains)

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
        return self._value

    @value.setter
    def value(self, v: Optional[Any]) -> None:
        if not VarDomains:
            raise ValueError("VarDomains is not defined. Load domains using load_domains() before setting variable values.")
        if self.domain not in VarDomains:
            raise ValueError(f"Domain {self.domain!r} is not defined in VarDomains.")
        if v is not None and v not in VarDomains[self.domain]:
            raise ValueError(f"{v!r} not in domain {VarDomains[self.domain]}")
        self._value = v

    def __call__(self, index: Optional[int] = None) -> Optional[Any]:
        """
            Enumerate valid values by index.
        """
        if not index:
            return self._value

        return VarDomains[self.domain][index] if index < len(VarDomains[self.domain]) else None

    def __str__(self) -> Any:
        if not self._value:
            return f"∅  ∈ {VarDomains[self.domain]}"
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

    def get_entities(self, key: str | Type[Entity]) -> Entity | List[Entity]:
        if isinstance(key, str):
            return self._entities_by_name[key]
        elif isinstance(key, type) and issubclass(key, Entity):
            return self._entities_by_type[key]
        else:
            raise KeyError(f"Invalid key type: {key}. Must be str or Type[Entity].")

@dataclass
class Condition:
    name: str
    src_entity_name: str
    src_var_type: str
    target: Any
    negate: bool = False

    def __call__(self, src_entity: Entity, target_entity: Optional[Entity] = None) -> str:
        variables = vars(src_entity)
        var = variables.get(self.src_var_type)
        if var is None:
            return f"Condition {self.name} failed: {src_entity.name} has no variable of type {self.src_var_type}"
        if var.value != target_entity.name if target_entity else self.target:
            return f"Condition {self.name} failed: {src_entity.name}_{self.src_var_type}={var.value} != {target_entity}"
        return f"Condition {self.name} passed: {src_entity.name}_{self.src_var_type}={var.value} == {target_entity}"

@dataclass
class Effect:
    name: str
    src_entity_name: str
    src_var_type: str
    target: Any

    def __call__(self, src_entity: Entity, target_entity: Optional[Entity] = None) -> str:
        variables = vars(src_entity)
        var = variables.get(self.src_var_type)
        if var is None:
            return f"Effect {self.name} failed: {src_entity.name} has no variable of type {self.src_var_type}"
        try:
            var.value = target_entity.name if target_entity else self.target
            return f"Effect {self.name} applied: {src_entity.name}_{self.src_var_type} set to {var.value}"
        except ValueError as e:
            return f"Effect {self.name} failed to apply: {e}"

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

    def check(self, param_entities: Dict[str, Entity]) -> bool:
        self._type_check(param_entities)
        entities = {ent_name: param_entities[ent_name] for ent_name in self.params}

        for cond in self.preconditions:
            src_entity = entities[cond.src_entity_name]
            target = entities[cond.target] if isinstance(cond.target, str) else None

            if not cond(src_entity, target):
                return False
        self._checked = True
        return True

    def execute(self, param_entities: Dict[str, Entity]) -> None:
        if self._checked is None:
            raise RuntimeError(f"Action {self.name} not checked before execution.")
        elif not self._checked:
            raise RuntimeError(f"Action {self.name} preconditions not satisfied. Cannot execute.")

        self._type_check(param_entities)
        entities = {ent_name: param_entities[ent_name] for ent_name in self.params}

        for eff in self.effects:
            src_entity = entities[eff.src_entity_name]
            target_entity = entities.get(eff.target) if isinstance(eff.target, str) else None
            eff(src_entity, target_entity)

    def __str__(self) -> str:
        return f"({self.name}])"

@dataclass
class LinkedState:
    state_id: int
    state: State
    state_type: StateStatus = StateStatus.ALIVE
    parent: Optional[Tuple[str, 'LinkedState']] = None
    branches_to_explore: List[Tuple[str, 'LinkedState']] = field(default_factory=list)
    children: List[Tuple[str, 'LinkedState']] = field(default_factory=list)
    cost: float = 0.0

    def __hash__(self) -> FrozenSet:
        return frozenset(self.state.items())

    def __eq__(self, other: 'LinkedState') -> bool:
        return frozenset(self.state.items()) == frozenset(other.state.items())

    def __str__(self) -> str:
        return f"State {self.state_id} ({self.state_type.name}): {self.state}"


@dataclass
class World:
    # Maybe create another dataclass that can query for entities with multiple keys, like EntityType and EntityName
    # Entity name queries a single entity, while EntityType queries for all entities of that type
    entities: Entities
    states: List[State] = field(default_factory=list)
    goal_state: State = field(default_factory=dict)

    @property
    def current_state(self) -> State:
        return self.states[-1] if self.states else {}

    def update_state(self) -> State:
        if self.states != []:
            new_state = deepcopy(self.current_state)
        else:
            new_state = {}

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