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
        with self.coordinator.coordinator_lock:
            assert self.coordinator.start_road is not None
            assert self.coordinator.other_croad is not None

            rospy.logdebug(f"[stopsign] my road: {self.coordinator.start_road.name}, other guy: {self.coordinator.other_croad}")
            rospy.logdebug(f"[stopsign] Do I have prio? "
                          f"{self._is_to_the_right(me=self.coordinator.start_road.name, other=self.coordinator.other_croad)}")
            rospy.logdebug(f"[stopsign] Have I stopped yet? {self.coordinator.stopped_topic_rcvd}")

            return self._is_to_the_right(me=self.coordinator.start_road.name,
                                         other=self.coordinator.other_croad) \
                   and self.coordinator.stopped_topic_rcvd
