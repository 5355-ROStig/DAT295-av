# coding=utf-8
"""
A Marker for displaying zones in RViz.

Basically a large, semi-transparent rectangle.

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
from visualization_msgs.msg import Marker


class ZoneMarker:
    def __init__(self, zone_id, zone_type, origin, width, height, color, y_mirroring):
        """Create a marker for a zone.

        :param zone_id: The ID of the zone. Reusing IDs replaces existing markers.
        :type zone_id: int
        :param zone_type: The type of the zone, such as "connection_zone".
                          Used for RViz namespace.
        :type zone_type: str
        :param origin: (x,y)-tuple of the lower-left corner of the marker.
        :type origin: tuple(int, int)
        :param width: The width of the zone
        :type width: int
        :param height: The height of the zone
        :type height: int
        :param color: The color of the zone in (r,g,b) format, all field are [0,255]
        :type color: tuple(int, int, int)
        :param y_mirroring: Size of map in y-direction, used to "mirror" the y coordinate.
        """
        self._id = zone_id
        self.zone_type = zone_type
        self.position = origin
        self.width = width
        self.height = height

        self.color = color

        self.mirroring_offset = y_mirroring

        self.marker = self._create_marker()

    def _create_marker(self):
        """Create a rectangular marker"""
        zone = Marker()
        # Draw zones directly on the map, no transform needed
        zone.header.frame_id = "map"
        zone.header.stamp = rospy.Time(0)

        # Creating a new marker with an already used ID will replace the existing
        # marker with the newer one.
        zone.ns = self.zone_type
        zone.id = self._id

        zone.type = Marker.CUBE
        zone.action = Marker.ADD

        # Pose of marker on the map
        # Position is apparently the center of the marker, offset it to "origin" corner
        zone.pose.position.x = self.position[0] + (self.width / 2)
        # Subtract width here, instead of adding. Y-axis is flipped, remember? :)
        # No?
        zone.pose.position.y = self.mirroring_offset + self.position[1] + (self.height / 2)
        zone.pose.position.z = 100
        zone.pose.orientation.x = 0
        zone.pose.orientation.y = 0
        zone.pose.orientation.z = 0
        zone.pose.orientation.w = 1

        # Set the width and height as size.
        zone.scale.x = self.width
        zone.scale.y = self.height
        zone.scale.z = 100  # Arbitrary z-"depth"

        # Set color
        zone.color.r = self.color[0] / 255.0
        zone.color.g = self.color[1] / 255.0
        zone.color.b = self.color[2] / 255.0
        zone.color.a = 0.7

        # Immortal marker
        zone.lifetime = rospy.Duration(0)

        return zone
