"""
A class containing RViz markers for trucks.
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
import random
import rospy
import tf_conversions
from visualization_msgs.msg import Marker
class TruckMarker:
    def __init__(self, truck_id):
        """Create a Marker, an object that can be drawn by RViz."""
        self._id = truck_id
        # Dimensions of the physical truck model in the lab, measured in mm.
        # These were measured in 2017 or 2018, we have no idea which lengths
        # they actually are :)
        # Some lengths from front to back of the cab
        self.cab_length1 = 95
        self.cab_length2 = 220
        self.cab_length3 = 110
        # Cab width
        self.cab_width = 180
        # Some lengths from front to back of the trailer
        self.trailer_length1 = 75
        self.trailer_length2 = 445 + 102.5 + 85
        self.trailer_length3 = 0
        # Trailer width
        self.trailer_width = 180
        # Create Markers for use with RViz
        self.cab = self._create_cab_marker()
        self.trailer = self._create_trailer_marker()
        self.label = self._create_label_marker()
    @property
    def cab_length(self):
        """The total length of the cab."""
        return self.cab_length1 + self.cab_length2 + self.cab_length3
    @property
    def trailer_length(self):
        """The total length of the trailer."""
        return self.trailer_length1 + self.trailer_length2 + self.trailer_length3
    def _create_cab_marker(self):
        """Create a marker for the truck's cab."""
        cab = Marker()
        cab.header.frame_id = "cab" + str(self._id)
        cab.header.stamp = rospy.Time(0)
        # Sending a new marker with the same namespace and id will overwrite the old one.
        cab.ns = "truck"
        cab.id = self._id
        cab.type = Marker.CUBE
        cab.action = Marker.ADD
        # Position is always at origin of the frame
        cab.pose.position.x = 0
        cab.pose.position.y = 0
        cab.pose.position.z = 0  # We're only working in 2D
        # Orientation is always identity
        cab.pose.orientation.x = 0
        cab.pose.orientation.y = 0
        cab.pose.orientation.z = 0
        cab.pose.orientation.w = 1
        # Set size
        cab.scale.x = self.cab_length
        cab.scale.y = self.cab_width
        cab.scale.z = 220  # Arbitrary height
        # Set color
        cab.color.r = random.random()
        cab.color.g = random.random()
        cab.color.b = random.random()
        cab.color.a = 0.9
        # Never-ending lifetime
        cab.lifetime = rospy.Duration(0)
        return cab
    def _create_trailer_marker(self):
        """Create a marker for the truck's trailer."""
        trailer = Marker()
        trailer.header.frame_id = "trailer" + str(self._id)
        trailer.header.stamp = rospy.Time(0)
        trailer.ns = "trailer"
        trailer.id = self._id
        trailer.type = Marker.CUBE
        trailer.action = Marker.ADD
        # Position is always at origin of the frame
        trailer.pose.position.x = 0
        trailer.pose.position.y = 0
        trailer.pose.position.z = 0  # We're only working in 2D
        # Orientation is always identity
        trailer.pose.orientation.x = 0
        trailer.pose.orientation.y = 0
        trailer.pose.orientation.z = 0
        trailer.pose.orientation.w = 1
        # Set size
        trailer.scale.x = self.trailer_length
        trailer.scale.y = self.trailer_width
        trailer.scale.z = 110  # Arbitrary height
        # Set color
        trailer.color.r = random.random()
        trailer.color.g = random.random()
        trailer.color.b = random.random()
        trailer.color.a = 0.9
        # Never-ending lifetime
        trailer.lifetime = rospy.Duration(0)
        return trailer
    def _create_label_marker(self):
        """Create a marker to label the truck with its ID"""
        label = Marker()
        # Put this marker in the same frame as the cab
        label.header.frame_id = "cab" + str(self._id)
        label.header.stamp = rospy.Time(0)
        label.ns = "truck_label"
        label.id = self._id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        # Place text on the middle of the cab
        label.pose.position.x = 0
        label.pose.position.y = 0
        label.pose.position.z = 0
        # The text size is only scaled by the z-value
        label.scale.z = 400
        # Set color to white
        label.color.r = 1
        label.color.g = 1
        label.color.b = 1
        label.color.a = 0.9
        # Set text to the truck ID
        label.text = str(self._id)
        # Never-ending lifetime
        label.lifetime = rospy.Duration(0)
        return label