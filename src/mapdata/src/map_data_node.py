#!/usr/bin/env python3
"""
A ROS node to provide other nodes with information about the static environment.

Copyright (c) 2020, Thomas Alexandersson
Copyright (c) 2020, Farzad Besharati
Copyright (c) 2020, Johannes Gustavsson
Copyright (c) 2020, Martin Hilgendorf
Copyright (c) 2020, Ivan Lyesnukhin
Copyright (c) 2020, Jian Shin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rospy

from map_data_reader import load_map

from environment import Map
from nav_msgs.msg import OccupancyGrid
from mapdata.srv import GetIntersection
from mapdata.msg import Position, RoadSection


class MapData:
    def __init__(self):
        rospy.init_node('map_data')

        map_name = rospy.get_param("~map_name")

        self.map_data: Map = load_map(map_name)
        rospy.loginfo(f"Loaded map '{map_name}'")

        map_data_topic = "rviz_map"
        rospy.loginfo(f"Publishing map occupancy grid data to /{map_data_topic}.")
        self.map_pub = rospy.Publisher(map_data_topic, OccupancyGrid,
                                       queue_size = 1, latch = True)
        self.map_pub.publish(self.map_data.get_occupancy_grid())

        # Scenario 
        scenario = rospy.get_param("~scenario")
        self.intersection = self._four_way_intersection(scenario)

        rospy.loginfo(f"Starting service for intersection data")
        rospy.Service("intersection_data", GetIntersection, self.intersection_data_handler)

    @staticmethod
    def _four_way_intersection(scenario):
        """
        Models the four way intersection to be used in experiments.
        """

        # Below is EG5355
        #
        # y x ->
        # |
        # V
        #
        # We use this notation for orientation
        #    +-------+
        #    |       |
        #    |   N   |  windows
        #    | W + E |
        # door   S   |
        #    +--------

        # Distances are given in mm
        road_length = 1000
        offset = 300

        # Measurements from the room (blue tape corners) they don't make sense
        # without the whiteboard image from Slack
        P0 = Position(x=2584, y=6891)
        P1 = Position(x=2591, y=6106)
        P2 = Position(x=1785, y=6106)
        P3 = Position(x=1795, y=6891)

        # Warning: copy-pasta programming :o
        # North road section
        rs_north = RoadSection()
        rs_north.name = "N"
        rs_north.left = P1
        rs_north.right = P2
        rs_north.length = road_length
        rs_north.stopline_offset = offset
        
        # South road section
        rs_south = RoadSection()
        rs_south.name = "S"
        rs_south.left = P3
        rs_south.right = P0
        rs_south.length = road_length
        rs_south.stopline_offset = offset

        # West road section
        rs_west = RoadSection()
        rs_west.name = "W"
        rs_west.left = P2
        rs_west.right = P3
        rs_west.length = road_length
        rs_west.stopline_offset = offset

        # East road section
        rs_east = RoadSection()
        rs_east.name = "E"
        rs_east.left = P0
        rs_east.right = P1
        rs_east.length = road_length
        rs_east.stopline_offset = offset

        # Get the constants
        PRIORITY_ROAD = rs_north.PRIORITY_ROAD
        GIVE_WAY = rs_north.GIVE_WAY
        STOP_SIGN = rs_north.STOP_SIGN
        TRAFFIC_LIGHT = rs_north.TRAFFIC_LIGHT

        # Print error message just in case someone made a typo
        if scenario not in ["scenario1", "scenario2", "scenario3"]:
            rospy.logerr(f"Scenario name {scenario} not found.")

        if scenario == "scenario1":
            # Huvudled N <-> S
            rs_north.priority_sign = PRIORITY_ROAD
            rs_south.priority_sign = PRIORITY_ROAD
            # Väjningsplikt E <-> W
            rs_east.priority_sign = GIVE_WAY
            rs_west.priority_sign = GIVE_WAY
        elif scenario == "scenario2":
            rs_north.priority_sign = STOP_SIGN
            rs_south.priority_sign = STOP_SIGN
            rs_east.priority_sign = STOP_SIGN
            rs_west.priority_sign = STOP_SIGN
        elif scenario == "scenario3":
            rs_north.priority_sign = TRAFFIC_LIGHT
            rs_south.priority_sign = TRAFFIC_LIGHT
            rs_east.priority_sign = TRAFFIC_LIGHT
            rs_west.priority_sign = TRAFFIC_LIGHT

        return GetIntersection(north=rs_north,
                               west=rs_west,
                               south=rs_south,
                               east=rs_east)


    def intersection_data_handler(self, data):
      """
      Handles requests to the intersection data service.
      """
      rospy.loginfo("Received request for intersection data. Handling...")
      return self.intersection


if __name__ == '__main__':
    MapData()
    rospy.spin()
