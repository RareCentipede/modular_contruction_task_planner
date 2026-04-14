from dataclasses import dataclass, field
from typing import Any, Dict, Optional

type State = Dict['Variable', Optional[Any]]

BOOL_VAR_DOMAIN = (True, False)
VarDomains = {'bool': BOOL_VAR_DOMAIN}

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