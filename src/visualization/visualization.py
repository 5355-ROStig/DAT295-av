#!/usr/bin/env python
"""
The main visualization code for the 2020 project.
Written in Python 2, because ROS' tf library is not yet compatible with
Python 3, as of ROS Melodic and Spring 2020. Apparently, the next version
of ROS (Noetic) should support Python 3.
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
import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker
from custom_msgs.msg import Path, Position, TruckState
from custom_msgs.srv import GetCDM
from track_marker import TrackMarker
from truck_marker import TruckMarker
from zone_marker import ZoneMarker
class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        # Get number of trucks in the system
        self.truck_count = rospy.get_param(rospy.search_param("trucks"))
        # Get the environment map as an occupancy grid
        self.map_grid = self._retrieve_map_grid()
        # Read from the MapMetaData stored in the occupancy grid
        self.map_scale = self.map_grid.info.resolution
        self.map_height = self.map_grid.info.height * self.map_scale
        self.map_width = self.map_grid.info.width * self.map_scale
        # Create Truck markers for each truck
        self.truck_markers = {}
        for truck_id in range(self.truck_count):
            self.truck_markers[truck_id] = TruckMarker(truck_id)
        # Publish map to RViz, latch the latest message on the topic so we
        # don't have to wait for RViz to start, else the message would be lost.
        self.map_pub = rospy.Publisher('rviz_map', OccupancyGrid,
                                       queue_size = 1, latch = True)
        # Send the map to rviz
        self.map_pub.publish(self.map_grid)
        # Publish truck markers to RViz
        self.truck_pub = rospy.Publisher('rviz_trucks', Marker, queue_size = 10)
        # Publish reference path markers to rviz
        self.ref_path_pub = rospy.Publisher('rviz_ref_path', Marker, queue_size = 10)
        # Set up publishers for initial poses to trucks
        self.init_pose_pubs = {truck_id: rospy.Publisher('truck' + str(truck_id) + '/initial_pose',
                                                         TruckState, queue_size = 10)
                               for truck_id in range(self.truck_count)}
        # Set up publishers for destinations to trucks
        self.dest_pubs = {truck_id: rospy.Publisher('truck' + str(truck_id) + '/destination',
                                                    Position, queue_size = 10)
                          for truck_id in range(self.truck_count)}
        # Publish CDM zone markers to RViz
        self.cdm_zone_pub = rospy.Publisher('rviz_cdm_zones', Marker, queue_size = 10)
        # Subscribe to all truck state topics
        for truck_id in range(self.truck_count):
            rospy.Subscriber('truck' + str(truck_id) + '/truck_state', TruckState, self.truck_state_handler_new)
        # Subscribe to 2d pose estimate topic for setting truck pose from rviz
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.set_initial_pose_handler)
        self.init_pose_id = self._looping_truck_id_gen()
        # Subscribe to 2D Nav Goal topic for truck destinations
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.set_destination_handler)
        self.dest_id_gen = self._looping_truck_id_gen()
        # Subscribe to reference path topic from path planner
        rospy.Subscriber('reference_path', Path, self.reference_path_handler)
        # Wait until rviz is listening, then publish all truck markers
        rospy.loginfo("Visualizer waiting until rviz has started...")
        if self.wait_for_subscribers(self.truck_pub, timeout = 10, rate = 2):
            rospy.loginfo("...rviz started, sending all markers")
            for truck in self.truck_markers.values():
                self.truck_pub.publish(truck.cab)
                self.truck_pub.publish(truck.trailer)
                self.truck_pub.publish(truck.label)
        # Request the CDM data from the map data server
        self.cdm_data = self._retrieve_cdm_data()
        if self.wait_for_subscribers(self.cdm_zone_pub, timeout = 10, rate = 2):
            self.draw_critical_sections()
            self.draw_connection_zones()
    @staticmethod
    def _retrieve_map_grid():
        """
        Fetch the OccupancyGrid from the map data server.
        This is the closest thing we can get to drawing an image
        of the map in RViz.
        """
        rospy.wait_for_service('map_occupancy_grid')
        map_grid = rospy.ServiceProxy('map_occupancy_grid', GetMap)
        try:
            response = map_grid()
            rospy.loginfo("Received Occupancy Grid")
            map_img = response.map
        except rospy.ServiceException as e:
            rospy.logerr("Error while contacting map data server: ", e)
            return
        return map_img
    @staticmethod
    def _retrieve_cdm_data():
        """Fetch the CDM data about critical sections from the map data server."""
        rospy.wait_for_service('cdm_data')
        cdm_data_service = rospy.ServiceProxy('cdm_data', GetCDM)
        try:
            cdm_data = cdm_data_service()
        except rospy.ServiceException as e:
            rospy.logerr("Error while fetching CDM data from map data server:", e)
            return
        return cdm_data.sections
    @staticmethod
    def wait_for_subscribers(topic_publisher, timeout, rate = 2):
        """
        Wait until the given topic has at least 1 subscriber.
        Returns True if the topic has more than 1 subscriber, or
        False if the wait timed out (i.e. still no subscribers).
        :param topic_publisher: A Publisher for the topic that is waiting for
                                subscribers
        :type topic_publisher: rospy.Publisher
        :param timeout: Maximum time in seconds until the wait should time out.
        :type timeout: int
        :param rate: The frequency in Hz at which to check for subscribers.
        :type rate: int
        :rtype: bool
        """
        # Return immediately if there are subscribers already
        if topic_publisher.get_num_connections() > 0:
            return True
        rate = rospy.Rate(rate)
        timeout = rospy.Duration(timeout)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < timeout:
            rate.sleep()
            if topic_publisher.get_num_connections() > 0:
                break
        return topic_publisher.get_num_connections() > 0
    def truck_state_handler_new(self, truck_state):
        """Transform frames instead of markers"""
        who = truck_state.id
        if who not in self.truck_markers:
            print "Received update for truck with unknown id %d", who
        model = self.truck_markers[who]
        cab_x, cab_y, trailer_middle_x, trailer_middle_y = self.calculate_positions(truck_state, model)
        # Generate cab transform
        cab_frame = model.cab.header.frame_id
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = cab_frame
        t.transform.translation.x = cab_x
        t.transform.translation.y = self.map_height - cab_y
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -truck_state.theta1)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.truck_pub.publish(model.cab)
        self.truck_pub.publish(model.label)
        tf2_ros.TransformBroadcaster().sendTransform(t)
        # Generate trailer transform
        trailer_frame = model.trailer.header.frame_id
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = trailer_frame
        t.transform.translation.x = trailer_middle_x
        t.transform.translation.y = self.map_height - trailer_middle_y
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -truck_state.theta2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.truck_pub.publish(model.trailer)
        tf2_ros.TransformBroadcaster().sendTransform(t)
    @staticmethod
    def calculate_positions(truck_state, model):
        """
        Returns positions and orientations needed for transforming cab and
        trailer frames from a truck_state update.
        Transcribed and cleaned up (a little) from 2017 project code.
        Check the 2017 report for what this code actually does, they didn't
        document it. We used some slightly better variable names, although
        they are also mainly just guesses :)
        Basically, it calculates the positions of certain parts of the truck...
        Variables names (original -> now), these are just guesses though:
            xf, yf -> x_front, y_front
            jx, jy -> joint_x, joint_y
            xtf, ytf -> trailer_front_x, trailer_front_y
            mtx, mty -> trailer_middle_x, trailer_middle_y
        """
        x = truck_state.p.x
        y = truck_state.p.y
        x_front = x + model.cab_length1 * math.cos(truck_state.theta1)
        cab_x = x_front - model.cab_length * 0.5 * math.cos(truck_state.theta1)
        y_front = y + model.cab_length1 * math.sin(truck_state.theta1)
        cab_y = y_front - model.cab_length * 0.5 * math.sin(truck_state.theta1)
        # Joint is the 'hinge' between cab and trailer
        joint_x = x - model.cab_length2 * math.cos(truck_state.theta1)
        joint_y = y - model.cab_length2 * math.sin(truck_state.theta1)
        trailer_front_x = joint_x + model.trailer_length1 * math.cos(truck_state.theta2)
        trailer_front_y = joint_y + model.trailer_length1 * math.sin(truck_state.theta2)
        trailer_middle_x = trailer_front_x - 0.5 * model.trailer_length * math.cos(truck_state.theta2)
        trailer_middle_y = trailer_front_y - 0.5 * model.trailer_length * math.sin(truck_state.theta2)
        return cab_x, cab_y, trailer_middle_x, trailer_middle_y
    def _looping_truck_id_gen(self):
        """Yields truck IDs sequentially, wrapping after the last ID."""
        truck_id = 0
        while True:
            yield truck_id
            truck_id += 1
            truck_id %= self.truck_count
    def set_initial_pose_handler(self, data):
        """
        Handler for setting initial pose on trucks via 2D Pose Estimate tool.
        Sets Pose on trucks one at a time, each call will set the next truck
        until all trucks have been given a pose, at which point the next truck
        to set a pose on will be the first one again.
        """
        who = self.init_pose_id.next()
        rospy.loginfo("Received initial pose message for truck %d" % who)
        position = data.pose.pose.position
        quat = data.pose.pose.orientation
        new_state = TruckState()
        new_state.p = Position(position.x, self.map_height - position.y)
        angle = tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        new_state.theta1 = new_state.theta2 = -angle[2]
        self.init_pose_pubs[who].publish(new_state)
    def set_destination_handler(self, data):
        """
        Handler for setting destinations on trucks with the 2D Nav Goal tool.
        Sets the destination for one truck at a time in order of ID.
        :type data: PoseStamped
        """
        who = self.dest_id_gen.next()
        rospy.loginfo("Received new destination for truck %d" % who)
        pos = data.pose.position
        position = Position(pos.x, self.map_height - pos.y)
        self.dest_pubs[who].publish(position)
    def reference_path_handler(self, data):
        """
        Handler for transforming received reference paths into markers for rviz.
        :param data: The Path to draw in Rviz
        :type data: Path
        """
        line_marker = TrackMarker(data.id, data.path, self.map_height)
        self.ref_path_pub.publish(line_marker.line)
    def draw_connection_zones(self):
        """
        Draw a rectangular marker on each connection zone of the CDM system.
        """
        zone_id_counter = 0
        for section in self.cdm_data:
            for zone in section.connection_zones:
                # Show connection zones in a nice orange color
                zone_marker = ZoneMarker(zone_id = zone_id_counter, zone_type = 'connection_zone',
                                         origin = (zone.pos.x, zone.pos.y), width = zone.width, height = zone.height,
                                         color = (255, 170, 0), y_mirroring = self.map_height)
                zone_id_counter += 1
                self.cdm_zone_pub.publish(zone_marker.marker)
    def draw_critical_sections(self):
        """
        Draw a rectangular marker on each critical section of the CDM system.
        """
        section_id_counter = 0
        for section in self.cdm_data:
            # Show critical section in a nice pink color
            section_marker = ZoneMarker(zone_id = section_id_counter, zone_type = 'critical_section',
                                        origin = (section.pos.x, section.pos.y), width = section.width,
                                        height = section.height, color = (255, 85, 127), y_mirroring = self.map_height)
            section_id_counter += 1
            self.cdm_zone_pub.publish(section_marker.marker)
if __name__ == '__main__':
    Visualizer()
    rospy.spin()