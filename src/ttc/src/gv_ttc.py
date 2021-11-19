#!/usr/bin/env python3
import rospy

import numpy as np
import array
import math
from datetime import datetime
import collections

from gv_client.msg import GulliViewPosition

# Constants
NUM_SAMPLES = 2
ROBOT_RADIUS = 0.25

# Variables
robots = {}

class Robot:
    def __init__(self, name):
        self.name = name
        self.buf = collections.deque(maxlen=NUM_SAMPLES) # historic positions
        self.v = np.array([0, 0]) # current velocity
        self.p = np.array([1, 1]) # current position

    def receivePosition(self, p):
        self.buf.append((datetime.now(), p))
        self.p = p
        self.updateVel()
        #print("New v for", self.name, "is", self.v)

    def updateVel(self):
        velSum = np.array([0, 0])
        t = 0
        p = np.array([0, 0])

        for val in self.buf:
            if t != 0:
                dt = (val[0] - t).total_seconds()
                dp = val[1] - p
                velSum = velSum + (dp / dt)
            t = val[0]
            p = val[1]

        self.v = velSum / self.buf.maxlen

    def collisionCheck(self):
        global robots

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


def ttc(r1, r2):
    global r
    o = r2.p - r1.p
    d = r2.v - r1.v

    if np.dot(o, o) <= pow(2 * ROBOT_RADIUS, 2):
        return 0

    underSqrt = pow(2 * np.dot(o, d), 2) - 4 * np.dot(d, d) * (np.dot(o, o) - pow(2 * ROBOT_RADIUS, 2))
    if underSqrt >= 0:
        t1 = (-2*np.dot(o, d) + math.sqrt(underSqrt)) / (2 * np.dot(d, d))
        #print("t1=", t1)
        t2 = (-2*np.dot(o, d) - math.sqrt(underSqrt)) / (2 * np.dot(d, d))
        #print("t2=", t1)
        return min(t1, t2)
    else:
        #print("No collision")
        return float("inf")

def callback(pos):
    global robots
    p = np.array([pos.x/1000, pos.y/1000])
    robot = robots.get(pos.tagId)

    if robot is None:
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
