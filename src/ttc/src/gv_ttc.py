#!/usr/bin/env python3
import rospy
from genpy.rostime import *

import numpy as np
import array
import math
from datetime import datetime, timedelta
import collections
from kalman import Kalman
from median import Median
from split_median import SplitMedian

from gv_client.msg import GulliViewPosition

# Constants
ROBOT_RADIUS = 0.25
TIMEOUT_SEC = 3
V_JITTER = 0.01

# Variables
robots = {}
collision = False
filter_creator = lambda _ : _

class Robot:
    def __init__(self, name, initPos, initTime):
        self.name = name
        self.filter: Filter = filter_creator(initPos)
        self.v = np.array([0, 0]) # current velocity
        self.p = initPos # current position
        self.lastReceive = initTime

    def receivePosition(self, p, timestamp):
        dt = (timestamp - self.lastReceive).to_sec()
        res = self.filter.newData(dt, p)
        self.p = np.array([res[0], res[1]])
        self.v = np.array([res[2], res[3]])
        if np.linalg.norm(self.v) < V_JITTER:
            self.v = np.array([0., 0.])
        self.lastReceive = timestamp

    # Check for collision against all other robots
    def collisionCheck(self):
        global collision

        t = float("inf")
        n = 0

        for k, v in robots.items():
            if v == self:
                continue

            n = n + 1

            res = ttc(self, v)

            if 0 <= res < t:
                t = res

        if n == 0:
            print("Nothing to collide with. v=", np.linalg.norm(self.v))
        elif t > 0:
            print(self.name, "Collision in ", t, " seconds. v=", np.linalg.norm(self.v))
        else:
            if not collision:
                print("Collision detected!")
            collision = True
            return
        collision = False

# Return time to collision in seconds between robot r1 and r2
# Value is float in [0, inf). 0 means robots are currently collided.
def ttc(r1, r2):
    o = r2.p - r1.p # Position of r2 in relation to r1
    d = r2.v - r1.v # Velocity of r2 in relation to velocity of r1

    # Note: dot(v, v) = ||v||^2

    # Check if already collided
    if np.dot(o, o) <= pow(2 * ROBOT_RADIUS, 2):
        return 0

    # Imagine a coordinate system rooted at the position of r1.
    # r2 will thus have position o and velocity d.
    # Now imagine r1 is a stationary circle with a radius 2*r, and r2 is a ray in the direction of d.
    # If the ray intersects the circle, a collision occurs.
    # r1 and r2, respectively, can be expressed as:
    # 2*r = ||p-c||
    # s(t) = o + td
    # let s(t) = p  =>  2*r = ||o+td-c||  =>  2*r = ||o+td|| (since center is 0,0)
    # Now, solve for t using the abc formula.

    underSqrt = pow(2 * np.dot(o, d), 2) - 4 * np.dot(d, d) * (np.dot(o, o) - pow(2 * ROBOT_RADIUS, 2))
    if underSqrt >= 0 and np.dot(d, d) > 0:
        t1 = (-2*np.dot(o, d) + math.sqrt(underSqrt)) / (2 * np.dot(d, d))
        t2 = (-2*np.dot(o, d) - math.sqrt(underSqrt)) / (2 * np.dot(d, d))
        return min(t1, t2)
    else:
        # Imaginary answer, no collision
        return float("inf")

def callback(gvPos):
    time = gvPos.header.stamp

    # Remove old robot positions
    delete = [k for k, v in robots.items() if v.lastReceive + Duration(TIMEOUT_SEC) < time]
    for k in delete:
        print("Robot", k, "is gone")
        del robots[k]

    # New position
    p = np.array([gvPos.x/1000, gvPos.y/1000]) # Convert from mm to m
    robot = robots.get(gvPos.tagId)

    if robot is None:
        # Add new robot
        robot = Robot(gvPos.tagId, p, time)
        robots[gvPos.tagId] = robot
    else:
        robot.receivePosition(p, time)
        robot.collisionCheck()

if __name__ == '__main__':
    rospy.init_node('ttc_listener', anonymous=True)

    # Choose filtering method
    filter_type = rospy.get_param('~filter', 'median')
    median_samples = rospy.get_param('~median_samples', 4)

    if filter_type == 'kalman':
        filter_creator = lambda initPos : Kalman(initPos)
    elif filter_type == 'median':
        filter_creator = lambda initPos : Median(initPos, median_samples)
    elif filter_type == 'split_median':
        filter_creator = lambda initPos : SplitMedian(initPos, median_samples)
    else:
        sys.exit("Invalid filter type")
    rospy.loginfo("Using " + filter_type + " filter ")

    # Start subscriber
    rospy.Subscriber('gv_positions', GulliViewPosition, callback)
    rospy.spin()
    rospy.loginfo("Bye!")
