#!/bin/env python3
import threading
from typing import List, Optional, Callable

import rospy

from gv_client.msg import GulliViewPosition
from mapdata.srv import GetIntersection
from mapdata.msg import RoadSection, StopLine

from phases.approach_phase import ApproachPhase
from phases.coordination_phase import CoordinationPhase
from phases.crossing_phase import CrossingPhase
from phases.leave_phase import LeavePhase
from phases.phase import Phase


class MissionPlannerNode:
    def __init__(self):
        rospy.init_node('mission_planner_node', anonymous=True)
        rospy.loginfo("Starting mission planner node")

        self.x: Optional[int] = None
        self.y: Optional[int] = None
        self._pos_lock = threading.Lock()

        scenario_param = rospy.get_param('~scenario')
        rospy.loginfo(f"Loading mission for requested scenario '{scenario_param}'")

        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)

        # Wait for initial position data
        try:
            rospy.loginfo("Awaiting initial position data")
            self._await(self._initial_position_received, timeout=30.0)
        except TimeoutError as e:
            rospy.logerr("Timeout while waiting for initial position")
            raise e
        rospy.loginfo(f"Initial position received: ({self.x}, {self.y})")

        # Query for environment/map data
        rospy.loginfo("Waiting for map data service")
        rospy.wait_for_service('intersection_data')
        get_road_data = rospy.ServiceProxy('intersection_data', GetIntersection)
        try:
            rospy.loginfo("Querying map data service...")
            self.road_data = get_road_data()
        except rospy.ServiceException as e:
            rospy.logerr("Error while fetching road data from map data server", e)
            rospy.signal_shutdown()
        rospy.loginfo("Retrieved map data")

        # Figure out on which road we are located
        rospy.loginfo("Determining mission parameters")
        # TODO: Implement this part and replace the placeholders here
        self.start_road = RoadSection()

        # Figure out the other mission parameters
        # self.priority_sign = None
        self.destination_road = RoadSection()
        self.start_line = StopLine()
        self.stop_line = StopLine()

        rospy.loginfo("Generating mission phases")
        self.phases: List[Phase] = [
            ApproachPhase(start_road=self.start_road, start_line=self.start_line),
            CoordinationPhase(start_road=self.start_road),
            CrossingPhase(start_road=self.start_road, destination_road=self.destination_road,
                          stop_line=self.stop_line),
            LeavePhase(self.destination_road, self.stop_line)
        ]

        rospy.loginfo("")
        rospy.loginfo("========== MISSION SUMMARY ==========")
        rospy.loginfo(f"Travelling from: {self.start_road.name}, my priority sign is: {self.start_road.priority_sign}")
        rospy.loginfo(f"Destination: {self.destination_road.name}")
        rospy.loginfo("")
        rospy.loginfo("Mission phases:")
        for idx, phase in enumerate(self.phases, start=1):
            rospy.loginfo(f"  - Phase {idx}/{len(self.phases)}: {phase.name}")
        rospy.loginfo("========== END OF SUMMARY ==========")
        rospy.loginfo("")

        self.execute_mission()

    @staticmethod
    def _await(condition: Callable[[], bool], rate: int = 2, timeout: float = 10.0):
        """Block until a given condition is True.

        Raises TimeoutError if condition does not become true before timeout.

        :param condition: A callable taking no arguments and returning a falsy value
                          while the condition is not met, and a truthy value once it is.
        :param rate: The frequency in Hz at which the condition should be polled
        :param timeout: Number of seconds before action times out.
        """
        rate = rospy.Rate(rate)
        timeout = rospy.Duration(secs=timeout)

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < timeout:
            if condition():
                return
            rate.sleep()
        raise TimeoutError()

    def _initial_position_received(self) -> bool:
        with self._pos_lock:
            return self.x is not None and self.y is not None

    def _position_cb(self, position_msg):
        with self._pos_lock:
            self.x = position_msg.x
            self.y = position_msg.y

    def execute_mission(self):
        start_time = rospy.Time.now()
        for phase_idx, phase in enumerate(self.phases, start=1):
            rospy.loginfo(f"Beginning phase {phase_idx}/{len(self.phases)}: {phase.name}")
            phase.begin()

            rospy.loginfo("Running phase")
            while not phase.condition():
                phase.run()

            rospy.loginfo("Finishing phase")
            phase.finish()

        finish_time = rospy.Time.now()

        rospy.loginfo(f"Mission finished in {(finish_time - start_time).secs} seconds")


if __name__ == '__main__':
    mission_planner = MissionPlannerNode()

    rospy.spin()
