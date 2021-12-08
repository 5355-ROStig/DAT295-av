from coordination_strategies.strategy import CoordinationStrategy

import rospy


class StopRoad(CoordinationStrategy):
    def __init__(self, coordinator):
        super().__init__("Stop-sign", coordinator)

    def _is_to_the_right(self, me: str, other: str) -> bool:
        mapping = {
            'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'
        }
        return mapping[me] == other

    def has_priority(self) -> bool:
        assert self.coordinator.start_road is not None
        assert self.coordinator.other_croad is not None

        rospy.loginfo(f"[stopsign] my road: {self.coordinator.start_road.name}, other guy: {self.coordinator.other_croad}")
        rospy.loginfo(f"[stopsign] Do I have prio? {self._is_to_the_right(me=self.coordinator.start_road.name, other=self.coordinator.other_croad)}")

        return self._is_to_the_right(me=self.coordinator.start_road.name,
                                     other=self.coordinator.other_croad)
