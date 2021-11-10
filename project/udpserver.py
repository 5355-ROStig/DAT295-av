#!/usr/bin/python3
import rospy
from socket import *
from geometry_msgs.msg import Twist

s = socket(AF_INET, SOCK_DGRAM)
s.bind(('', 5005))

rospy.init_node("project")
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

while True:
    m = s.recvfrom(5005)
    m = m[0].decode("utf-8")
    if m == "STOP":
        print("Stop message received")
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x= 0
        twist.angular.y= 0
        twist.angular.z= 0
        publisher.publish(twist)

