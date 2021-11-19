#!/usr/bin/env python3
import rospy

import numpy as np
import array
import math
from datetime import datetime, timedelta
import collections

from gv_client.msg import GulliViewPosition

# Constants
NUM_SAMPLES = 2
ROBOT_RADIUS = 0.25
TIMEOUT_SEC = 3

# Variables
robots = {}

class Robot:
    def __init__(self, name):
        self.name = name
        self.buf = collections.deque(maxlen=NUM_SAMPLES) # historic positions
        self.v = np.array() # current velocity
        self.p = np.array() # current position
        self.lastReceive = datetime.now()

    def receivePosition(self, p):
        self.p = p
        self.lastReceive = datetime.now()
        self.buf.append((self.lastReceive, p))
        self.__updateVel()

    # Calculate new velocity, averaging the last
    def __updateVel(self):
        velSum = np.array([0, 0])
        lastT = 0
        lastP = np.array([0, 0])

        for val in self.buf:
            if lastT != 0:
                dt = (val[0] - lastT).total_seconds()
                dp = val[1] - lastP
                velSum = velSum + (dp / dt)
            lastT = val[0]
            lastP = val[1]

        self.v = velSum / self.buf.maxlen

    # Check for collision against all other robots
    def collisionCheck(self):
        t = float("inf")

        for k, v in robots.items():
            if v == self:
                continue

            res = ttc(self, v)

            if 0 <= res < t:
                t = res

        if t > 0:
            print("Collision in ", t, " seconds")
        else:
            print("Collision detected!")

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

def callback(pos):
    # Remove old robot positions
    delete = [k for k, v in robots.items() if v.lastReceive + timedelta(seconds=TIMEOUT_SEC) < datetime.now()]
    for k in delete:
        print("Robot", k, "is gone")
        del robots[k]

    # New position
    p = np.array([pos.x/1000, pos.y/1000]) # Convert from mm to m
    robot = robots.get(pos.tagId)

    if robot is None:
        # Add new robot
        robot = Robot(pos.tagId)
        robots[pos.tagId] = robot
        robot.receivePosition(p)
    else:
        robot.receivePosition(p)
        robot.collisionCheck()

if __name__ == '__main__':
    rospy.init_node('ttc_listener', anonymous=True)
    rospy.Subscriber('gv_positions', GulliViewPosition, callback)
    rospy.spin()
    rospy.loginfo("Bye!")
