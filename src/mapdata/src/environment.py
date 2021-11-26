"""
A class to contain data and methods about the road network map.

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
from pathlib import Path

import cv2
import rospy
import yaml
from geometry_msgs.msg import Pose, Quaternion, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData

from cdm_section import CDMSection
import pickle
from os.path import isfile


class Map:
    def __init__(self, name: str, image: Path, image_scale: int ):
        self.name = name
        self._image_file = image.resolve()
        self.image = self._load_image(image)
        self.scale = image_scale
        self._occupancy_grid = None

    def __str__(self):
        return f"Map '{self.name}', scale: {self.scale}, image: '{self._image_file}'"

    @property
    def width(self):
        """Get actual width of the map"""
        return len(self.image[0]) * self.scale

    @property
    def height(self):
        """Get the actual height of the map"""
        return len(self.image) * self.scale

    @staticmethod
    def _load_image(path: Path):
        """Given a path to an image file, read it using openCV"""
        if not path.exists():
            raise FileNotFoundError(f"Could not find map image file: '{path.resolve()}'")
        if not path.is_file():
            raise IsADirectoryError(f"Could not load map image file, is a directory: '{path.resolve()}'")

        return cv2.imread(str(path))

    def get_occupancy_grid(self):
        """
        Converts the map image to an occupancy grid for RViz

        An occupancy grid is a matrix with a field for each pixel of the map:
            Value 100: Occupied (black)
            Value 0: Unoccupied (white)
            Anything else: Unknown (grey)
        """
        stamp = rospy.Time.now()

        height = len(self.image)
        width = len(self.image[0])

        oc = OccupancyGrid()
        oc.header.frame_id = "map"
        oc.header.stamp = stamp

        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0))

        oc.info = MapMetaData(stamp, self.scale, width, height, origin)

        # Speed up by using serialized occupancy grid data
        pickle_path = f"{self._image_file}.pickle"
        if isfile(pickle_path):
            rospy.loginfo("Pickled OccupancyGrid found")
            f = open(pickle_path, "rb")
            oc.data = pickle.load(f)
            f.close()
            rospy.loginfo("Sending pickled OccupancyGrid")
            return oc

        # If the grid is not cached, generate it
        if self._occupancy_grid is None:
            # Loop over all pixels, inverting the y-axis (height) to
            # produce a grid with (0,0) in the lower left corner.
            for idy, row in enumerate(self.image[::-1]):
                for idx, pixel in enumerate(row):
                    # Average the colour data
                    avg_color = sum(pixel) / len(pixel)

                    if avg_color == 0:
                        oc.data.append(100)
                    elif avg_color == 255:
                        oc.data.append(0)
                    else:
                        oc.data.append(50)
            # Cache the grid
            self._occupancy_grid = oc.data
        else:
            # Use cached grid
            oc.data = self._occupancy_grid

        rospy.loginfo("Pickling and sending OccupancyGrid...")
        f = open(pickle_path, "wb")
        pickle.dump(oc.data, f)
        f.close()
        return oc
