#!/usr/bin/python3
import rospy
from roswifibot.msg import IR
from geometry_msgs.msg import Twist
import time
import sys

rospy.init_node("IRlistener")
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

def listener_ir():
 rospy.Subscriber("IR", IR, ir_analyser)
 rospy.spin()

def ir_analyser(data):
 if (data.IR_front_left < 50 or data.IR_front_right < 50):
  print("I see a target")
  move(0)
  print ("Left IR: " + str(data.IR_front_left))
  print ("Right IR: " + str(data.IR_front_right))
 else:
  print ("No object detected")

def move(speed):
 twist.linear.x = float(speed)
 twist.linear.y = 0
 twist.linear.z = 0
 twist.angular.x = 0
 twist.angular.y = 0
 twist.angular.z = 0
 publisher.publish(twist)

time.sleep(1)
move(0.2)
listener_ir()

