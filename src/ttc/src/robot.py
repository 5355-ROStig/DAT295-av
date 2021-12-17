#!/usr/bin/env python3

import numpy as np
import array
import math
from filter import Filter
from datetime import datetime, timedelta
import collections
from median import Median
from split_median import SplitMedian

# Constants
ROBOT_RADIUS = 0.25
TIMEOUT_SEC = 3
V_JITTER = 0.01

# Variables
robots = {}
filter_creator = lambda _ : _

class Robot:
    def __init__(self, name, initPos, initTime):
        self.name = name
        self.filter: Filter = filter_creator(initPos)
        self.v = np.array([0, 0]) # current velocity
        self.p = initPos # current position
        self.lastReceive = initTime

    def receivePosition(self, p, timestamp):
        dt = (timestamp - self.lastReceive)
        res = self.filter.newData(dt, p)
        self.p = np.array([res[0], res[1]])
        self.v = np.array([res[2], res[3]])
        if np.linalg.norm(self.v) < V_JITTER:
            self.v = np.array([0., 0.])
        self.lastReceive = timestamp

    # Check for collision against all other robots, return ttc
    def collisionCheck(self):
        t = float("inf")
        n = 0

        for k, v in robots.items():
            if v == self:
                continue

            n = n + 1

            res = _ttc(self, v)

            if 0 <= res < t:
                t = res

        return t

# Return time to collision in seconds between robot r1 and r2
# Value is float in [0, inf). 0 means robots are currently collided.
def _ttc(r1, r2):
    o = r2.p - r1.p # Position of r2 in relation to r1
    d = r2.v - r1.v # Velocity of r2 in relation to velocity of r1

    # Note: dot(v, v) = ||v||^2

    # Check if already collided
    if np.dot(o, o) <= pow(2 * ROBOT_RADIUS, 2):
        return 0

    # Imagine a coordinate system rooted at the position of r1.
    # r2 will thus have position o and velocity d.
    # Now imagine r1 is represented by a stationary circle with a radius 2*r,
    # and r2 is a ray in the direction of d.
    # If the ray intersects the circle, a collision occurs.
    # r1 and r2, respectively, can be expressed as:
    # r1: 2*r = ||p-c||  (equation for a circle with center c and radius 2*r)
    # r2: s(t) = o + td  (equation for a ray s(t) with starting position o and normalized direction d)
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

# Receive an updated position. Return the robot object position is applied to.
def receive_pos(tagId, x, y, timestamp):
    # Remove old robot positions
    delete = [k for k, v in robots.items() if v.lastReceive + TIMEOUT_SEC < timestamp]
    for k in delete:
        print("Robot", k, "is gone")
        del robots[k]

    # New position
    p = np.array([x, y])
    robot = robots.get(tagId)

    if robot is None:
        # Add new robot
        robot = Robot(tagId, p, timestamp)
        robots[tagId] = robot
    else:
        robot.receivePosition(p, timestamp)
    
    return robot

