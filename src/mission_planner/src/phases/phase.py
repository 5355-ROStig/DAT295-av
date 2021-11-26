import typing
from abc import ABC, abstractmethod

if typing.TYPE_CHECKING:
    from mission_planner_node import MissionPlannerNode


class Phase(ABC):

    @abstractmethod
    def __init__(self, mission: "MissionPlannerNode"):
        self.mission = mission

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
