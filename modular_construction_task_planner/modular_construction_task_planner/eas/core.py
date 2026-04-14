from dataclasses import dataclass, field
from typing import Any, Dict, Optional, List, Type, Tuple

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