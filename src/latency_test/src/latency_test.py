#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from gv_client.msg import GulliViewPosition
from timeit import default_timer as timer
from typing import List, Optional, Callable

cmd_vel_pub = None
tag_id = None
start_pos = None
started = False

def _send_spd(speed: float):
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

def _await(condition: Callable[[], bool], rate: int = 2, timeout: float = 10.0):
    """Block until a given condition is True.

    Raises TimeoutError if condition does not become true before timeout.

    :param condition: A callable taking no arguments and returning a falsy value
                      while the condition is not met, and a truthy value once it is.
    :param rate: The frequency in Hz at which the condition should be polled
    :param timeout: Number of seconds before action times out.
    """
    rate = rospy.Rate(rate)
    timeout = rospy.Duration(secs=timeout)

    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < timeout:
        if condition():
            return
        rate.sleep()
    raise TimeoutError()

def _position_cb(position_msg):
    global start_pos, started

    if position_msg.tagId != tag_id:
        return

    p = np.array([position_msg.x/1000, position_msg.y/1000])

    if start_pos is None:
        start_pos = p
    elif np.linalg.norm(p-start_pos) > 0.002 and started:
        end = timer()
        print((end-start))
        started = False

if __name__ == '__main__':
    tag_id = 5
    rospy.init_node('latency_test_node', anonymous=True)
    rospy.Subscriber('gv_positions', GulliViewPosition, _position_cb)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    while not rospy.is_shutdown():
        start_pos = None
        _await(lambda: start_pos is not None)

        start = timer()
        started = True

        while started and not rospy.is_shutdown():
            _send_spd(0.25)
            rospy.sleep(0.1)
        _send_spd(0)
        rospy.sleep(1.)

