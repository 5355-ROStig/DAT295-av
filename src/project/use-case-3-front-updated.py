#!/usr/bin/python3
import rospy
import time
import sys
from socket import *
from geometry_msgs.msg import Twist
import threading


class Model(object):
    def __init__(self):
        self.currentSpeed = 0

    def updateSpeed(self, speed):
        self.currentSpeed = speed
        print(self.currentSpeed)

    def start(self):
        rospy.init_node("useCase3Front")
        publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print ("Inside the loop", self.currentSpeed)
            twist.linear.x = float(self.currentSpeed)
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            publisher.publish(twist)
            rate.sleep()
            if self.currentSpeed == 0:
                break

def speed_phases(model):
    print ('I AM IVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAN!!!!!!!!!!!!!!!!')
    model.updateSpeed(0.1)
    time.sleep(6)
    model.updateSpeed(0.5)
    time.sleep(6)
    model.updateSpeed(0.2)
    time.sleep(6)
    model.updateSpeed(-0.2)
    time.sleep(10)
    model.updateSpeed(0)


#we start movement
model = Model()

t = threading.Thread(target=speed_phases, args=[model])
t.start()

model.start()


