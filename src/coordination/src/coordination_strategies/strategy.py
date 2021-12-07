from abc import ABC, abstractmethod


class CoordinationStrategy(ABC):
    @abstractmethod
    def __init__(self, name: str):
        self.name = name

    def name(self) -> str:
        return self.name

    @abstractmethod
    def has_priority(self):
        ...
