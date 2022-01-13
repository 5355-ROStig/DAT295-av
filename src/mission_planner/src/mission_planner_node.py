#!/bin/env python3
from collections import namedtuple
from typing import List, Optional, Callable

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from gv_client.msg import GulliViewPosition
from mapdata.srv import GetIntersection
from mapdata.msg import RoadSection
from missions import MISSIONS

from phases.phase import Phase


Position = namedtuple('Position', ['x', 'y'])


class MissionPlannerNode:
    def __init__(self):
        rospy.init_node('mission_planner_node', anonymous=True)
        rospy.loginfo("Starting mission planner node")

        self.started = False
        rospy.Subscriber('/experiment_start', Empty, self._start_cb)
        try:
            rospy.loginfo("Awaiting start command from experiment controller")
            self._await(self._start_received, timeout=300.0)
        except TimeoutError as e:
            rospy.logerr("Timeout while waiting for start command")
            raise e

        self.pos: Optional[Position] = None

        scenario_param = rospy.get_param('/scenario')
        rospy.loginfo(f"Loading mission for requested scenario '{scenario_param}'")

        self.drive_speed = rospy.get_param('~speed')
        rospy.loginfo(f"Set drive speed to {self.drive_speed} m/s")

        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)

        self.go = False
        rospy.Subscriber('go', Empty, self._receive_go)

        # Wait for initial position data
        try:
            rospy.loginfo("Awaiting initial position data")
            self._await(self._initial_position_received, timeout=30.0)
        except TimeoutError as e:
            rospy.logerr("Timeout while waiting for initial position")
            raise e
        rospy.loginfo(f"Initial position received: ({self.pos.x}, {self.pos.y})")

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

        rospy.loginfo("Determining mission parameters")
        # Figure out which road we are starting from
        self.start_road = self._determine_starting_road()

        # Figure out the other mission parameters
        destinations = {
            'N': self.road_data.south,
            'S': self.road_data.north,
            'E': self.road_data.west,
            'W': self.road_data.east
        }

        self.destination_road = destinations[self.start_road.name]
        self.start_line = self._find_stopline(self.start_road)
        self.stop_line = self._find_stopline(self.destination_road)

        rospy.loginfo("Generating mission phases")
        self.phases: List[Phase] = [p(mission=self) for p in MISSIONS[scenario_param]]

        priority_sign_names = {
            RoadSection.PRIORITY_ROAD: "Priority Road",
            RoadSection.GIVE_WAY: "Give Way",
            RoadSection.STOP_SIGN: "Stop Sign",
            RoadSection.TRAFFIC_LIGHT: "Traffic light"
        }

        rospy.loginfo("")
        rospy.loginfo("========== MISSION SUMMARY ==========")
        rospy.loginfo(f"Travelling from: {self.start_road.name}, my priority sign is: "
                      f"{priority_sign_names[self.start_road.priority_sign]}")
        rospy.loginfo(f"Destination: {self.destination_road.name}")
        rospy.loginfo("")
        rospy.loginfo("Mission phases:")
        for idx, phase in enumerate(self.phases, start=1):
            rospy.loginfo(f"  - Phase {idx}/{len(self.phases)}: {phase.name}")
        rospy.loginfo("========== END OF SUMMARY ==========")
        rospy.loginfo("")

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.exit_pub = rospy.Publisher('exit', Empty, queue_size=1)
        self.stopped_pub = rospy.Publisher('stopped', Empty, queue_size=1)

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
        while rospy.Time.now() - start_time < timeout and not rospy.is_shutdown():
            if condition():
                return
            rate.sleep()
        raise TimeoutError()

    def _initial_position_received(self) -> bool:
        return self.pos is not None

    def _position_cb(self, position_msg):
        self.pos = Position(position_msg.x, position_msg.y)

    def _start_received(self) -> bool:
        return self.started is True

    def _start_cb(self, _):
        self.started = True

    def _receive_go(self, _):
        rospy.loginfo("Received go!")
        self.go = True

    def _determine_starting_road(self):
        for road_section in (self.road_data.north, self.road_data.south, self.road_data.east, self.road_data.west):
            if road_section.name == 'N':
                min_x = road_section.right.x
                max_x = road_section.left.x
                max_y = max(road_section.left.y, road_section.right.y)
                min_y = max_y - road_section.length
            elif road_section.name == 'S':
                min_x = road_section.left.x
                max_x = road_section.right.x
                min_y = min(road_section.left.y, road_section.right.y)
                max_y = min_y + road_section.length
            elif road_section.name == 'E':
                min_y = road_section.right.y
                max_y = road_section.left.y
                min_x = min(road_section.left.x, road_section.right.x)
                max_x = min_x + road_section.length
            elif road_section.name == 'W':
                min_y = road_section.left.y
                max_y = road_section.right.y
                max_x = max(road_section.left.x, road_section.right.x)
                min_x = max_x - road_section.length

            if (min_x <= self.pos.x <= max_x) and (min_y <= self.pos.y <= max_y):
                return road_section
        raise RuntimeError("Unable to determine starting road! "
                           "Make sure robot is positioned properly")

    @staticmethod
    def _find_stopline(road_section):
        if road_section.name == 'N':
            return min(road_section.left.y, road_section.right.y) - road_section.stopline_offset
        elif road_section.name == 'S':
            return min(road_section.left.y, road_section.right.y) + road_section.stopline_offset
        elif road_section.name == 'E':
            return min(road_section.left.x, road_section.right.x) + road_section.stopline_offset
        elif road_section.name == 'W':
            return min(road_section.left.x, road_section.right.x) - road_section.stopline_offset

    def execute_mission(self):
        start_time = rospy.Time.now()
        for phase_idx, phase in enumerate(self.phases, start=1):
            rospy.loginfo(f"Beginning phase {phase_idx}/{len(self.phases)}: {phase.name}")
            phase.begin()

            rospy.loginfo("Running phase")
            rate = rospy.Rate(10)
            while not phase.condition() and not rospy.is_shutdown():
                phase.run()
                rate.sleep()

            rospy.loginfo("Finishing phase")
            phase.finish()

        finish_time = rospy.Time.now()

        rospy.loginfo(f"Mission finished in {(finish_time - start_time).secs} seconds")


if __name__ == '__main__':
    while not rospy.is_shutdown():
        mission_planner = MissionPlannerNode()
        rospy.loginfo("Starting mission execution")
        mission_planner.execute_mission()
