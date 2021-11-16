#!/usr/bin/python3
import rospy
import time
import sys
from socket import *
from geometry_msgs.msg import Twist



rospy.init_node("project")
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

def move(speed):
 print("Stop message received")
 twist.linear.x = float(speed)
 twist.linear.y = 0
 twist.linear.z = 0
 twist.angular.x = 0
 twist.angular.y = 0
 twist.angular.z = 0
 publisher.publish(twist)


time.sleep(1)
move(0.2)
time.sleep(3)
move(0)

