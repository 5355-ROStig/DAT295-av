"""
A class to contain data about a CDM-enabled section of the map.

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
from collections import namedtuple
from threading import Lock
from typing import Dict

from mapdata.msg import CDMConnectionZone, Position
from mapdata.msg import CDMSection as CDMSectionMsg  # To avoid naming clash

connection_zone = namedtuple('connection_zone', ['origin', 'width', 'height'])

zone_type = {
    '3way': CDMSectionMsg.THREE_WAY_INTERSECTION,
    '4way': CDMSectionMsg.FOUR_WAY_INTERSECTION,
    '3way_roundabout': CDMSectionMsg.THREE_WAY_ROUNDABOUT
}

orientations = {
    'N': CDMSectionMsg.NORTH,
    'S': CDMSectionMsg.SOUTH,
    'E': CDMSectionMsg.EAST,
    'W': CDMSectionMsg.WEST
}


class CDMSection:
    __id_counter = 0
    __id_counter_lock = Lock()

    def __init__(self, section_data: Dict):
        """Create a critical section data object from the loaded data."""
        # Get a new unique ID for this section
        with CDMSection.__id_counter_lock:
            self._id = CDMSection.__id_counter
            CDMSection.__id_counter += 1

        self.section_type = section_data['type']
        self.position = (section_data['position']['x'], section_data['position']['y'])
        self.width = section_data['width']
        self.height = section_data['height']
        self.orientation = section_data['orientation']

        # Parse all connection zones
        self.conn_zones = [self._create_connection_zone(zone) for zone in
                           section_data['connection_zones']]

    def __repr__(self):
        return f"CDM id: {self._id}, type: {self.section_type} at {self.position} {self.width}x{self.height} facing " \
               f"{self.orientation}. Connection zones: {self.conn_zones}"

    def _create_connection_zone(self, conn_zone_data: Dict) -> connection_zone:
        """Create a connection zone tuple."""
        # Resolve the x and y coordinates of the zone, relative to the section origin
        absolute_x = self.position[0] + conn_zone_data['origin']['x']
        absolute_y = self.position[1] + conn_zone_data['origin']['y']

        return connection_zone(origin = (absolute_x, absolute_y), width = conn_zone_data['width'],
                               height = conn_zone_data['height'])

    @staticmethod
    def _conn_zone_to_ros(conn_zone: connection_zone):
        """Convert a connection zone tuple into a ROS message."""
        return CDMConnectionZone(pos = Position(*conn_zone.origin), width = conn_zone.width, height = conn_zone.height)

    def to_ros_message(self):
        """Convert this CDM section data object into a ROS message."""
        return CDMSectionMsg(self._id, Position(*self.position), zone_type[self.section_type],
                             self.width, self.height, orientations[self.orientation],
                             [CDMSection._conn_zone_to_ros(zone) for zone in self.conn_zones])
