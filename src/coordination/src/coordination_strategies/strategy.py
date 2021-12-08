import typing
from abc import ABC, abstractmethod

import rospy

if typing.TYPE_CHECKING:
    from coordination.src.coordination_node import CoordinationNode


class CoordinationStrategy(ABC):
    @abstractmethod
    def __init__(self, name: str, coordinator: 'CoordinationNode'):
        self.name = name
        self.coordinator = coordinator

    def name(self) -> str:
        return self.name

    def _is_to_the_right(self, me: str, other: str) -> bool:
        mapping = {
            'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'
        }
        return mapping[me] == other

    @abstractmethod
    def has_priority(self) -> bool:
        assert self.coordinator.start_road is not None
        assert self.coordinator.other_croad is not None

        rospy.loginfo(f"[stopsign] my road: {self.coordinator.start_road}, other guy: {self.coordinator.other_croad}")
        rospy.loginfo(f"[stopsign] Do I have prio? {self._is_to_the_right(me=self.coordinator.start_road, other=self.coordinator.other_croad)}")

        return self._is_to_the_right(me=self.coordinator.start_road,
                                     other=self.coordinator.other_croad)
