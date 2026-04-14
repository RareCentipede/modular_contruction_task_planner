from dataclasses import dataclass, field
from typing import Any, Dict, Optional, List, Type

type State = Dict['Variable', Optional[Any]]

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
class Condition:
    name: str
    src_entity_type: Type[Entity]
    src_var_type: str
    negate: bool = False

    def __call__(self, src_entity: Entity, expected_val: Any) -> str:
        variables = vars(src_entity)
        var = variables.get(self.src_var_type)
        if var is None:
            return f"Condition {self.name} failed: {src_entity.name} has no variable of type {self.src_var_type}"
        if var.value != expected_val:
            return f"Condition {self.name} failed: {src_entity.name}_{self.src_var_type}={var.value} != {expected_val}"
        return f"Condition {self.name} passed: {src_entity.name}_{self.src_var_type}={var.value} == {expected_val}"

@dataclass
class Effect:
    name: str
    src_entity_type: Type[Entity]
    target_var_type: str

    def __call__(self, target_entity: Entity, new_val: Any) -> str:
        variables = vars(target_entity)
        var = variables.get(self.target_var_type)
        if var is None:
            return f"Effect {self.name} failed: {target_entity.name} has no variable of type {self.target_var_type}"
        try:
            var.value = new_val
            return f"Effect {self.name} applied: {target_entity.name}_{self.target_var_type} set to {new_val}"
        except ValueError as e:
            return f"Effect {self.name} failed to apply: {e}"

@dataclass
class Action:
    name: str
    params: List[Type[Entity]]
    preconditions: List[Condition]
    effects: List[Effect]

    def check(self, param_entities: Dict[Type[Entity], Entity], target: Any) -> bool:
        entities = {ent_type: param_entities[ent_type] for ent_type in self.params}

        for cond in self.preconditions:
            entity = entities[cond.src_entity_type]
            if not cond(entity, target):
                return False
        return True

    def execute(self, param_entities: Dict[Type[Entity], Entity], target: Any) -> None:
        entities = {ent_type: param_entities[ent_type] for ent_type in self.params}

        for eff in self.effects:
            entity = entities[eff.src_entity_type]
            eff(entity, target)

    def __str__(self) -> str:
        return f"Action({self.name})"