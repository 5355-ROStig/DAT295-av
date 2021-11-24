from abc import ABC, abstractmethod


class Phase(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        ...

    @abstractmethod
    def begin(self):
        ...

    @abstractmethod
    def run(self):
        ...

    @abstractmethod
    def finish(self):
        ...

    @abstractmethod
    def condition(self) -> bool:
        ...
