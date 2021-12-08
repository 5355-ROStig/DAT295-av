import typing
from abc import ABC, abstractmethod

if typing.TYPE_CHECKING:
    from coordination.src.coordination_node import CoordinationNode


class CoordinationStrategy(ABC):
    @abstractmethod
    def __init__(self, name: str, coordinator: 'CoordinationNode'):
        self.name = name
        self.coordinator = coordinator

    def name(self) -> str:
        return self.name

    @abstractmethod
    def has_priority(self) -> bool:
        ...
