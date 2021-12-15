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
import robot

collision = False

def callback(gvPos):
    time = gvPos.header.stamp.to_sec()
    x = gvPos.x/1000
    y = gvPos.y/1000
    tagId = gvPos.tagId
    
    r = robot.receive_pos(tagId, x, y, time)
    ttc = r.collisionCheck()

    if len(robot.robots) <= 1:
        print(tagId, "Nothing to collide with. v=", np.linalg.norm(r.v))
    elif ttc > 0:
        print(tagId, "Collision in ", ttc, " seconds. v=", np.linalg.norm(r.v))
    else:
        if not collision:
            print("Collision detected!")
        collision = True
        return
    collision = False

if __name__ == '__main__':
    rospy.init_node('ttc_listener', anonymous=True)

    # Choose filtering method
    filter_type = rospy.get_param('~filter', 'median')
    median_samples = rospy.get_param('~median_samples', 4)

    if filter_type == 'kalman':
        robot.filter_creator = lambda initPos : Kalman(initPos)
    elif filter_type == 'median':
        robot.filter_creator = lambda initPos : Median(initPos, median_samples)
    elif filter_type == 'split_median':
        robot.filter_creator = lambda initPos : SplitMedian(initPos, median_samples)
    else:
        sys.exit("Invalid filter type")
    rospy.loginfo("Using " + filter_type + " filter ")

    # Start subscriber
    rospy.Subscriber('gv_positions', GulliViewPosition, callback)
    rospy.spin()
    rospy.loginfo("Bye!")
