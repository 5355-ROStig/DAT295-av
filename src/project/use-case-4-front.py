#!/usr/bin/python3
import rospy
import time
import sys
from socket import *
from geometry_msgs.msg import Twist
import threading
import json


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


def speed_broadcast(speed):
 UDP_IP = "192.168.1.255"
 UDP_PORT = 5005
 #MESSAGE = "SPEED " + str(speed)
 msg_speed = {
         'speed': speed,
         'timestamp': time.time()
         }
 msg_speed = json.dumps(msg_speed)
 for i in range(0,2):
  print("I AM SENDING " + str(speed))
  sock = socket(AF_INET, SOCK_DGRAM) # UDP
  sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
  sock.sendto(bytes(msg_speed, "utf-8"), (UDP_IP, UDP_PORT))




def speed_phases(model):
    speed = 0.1
    model.updateSpeed(speed)
    speed_broadcast(speed)
    time.sleep(6)
    speed = 0.5
    model.updateSpeed(speed)
    speed_broadcast(speed)
    time.sleep(6)
    speed = 0.2
    model.updateSpeed(speed)
    speed_broadcast(speed)
    time.sleep(6)
    speed = -0.2
    model.updateSpeed(speed)
    speed_broadcast(speed)
    time.sleep(10)
    speed = 0
    model.updateSpeed(speed)
    speed_broadcast(speed)


#we start movement
model = Model()

t = threading.Thread(target=speed_phases, args=[model])
t.start()

model.start()


