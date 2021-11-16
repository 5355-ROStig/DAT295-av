#!/usr/bin/python3
import rospy
import time
import sys
from socket import *
from geometry_msgs.msg import Twist



rospy.init_node("useCase1")
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

#we start movement
def move(speed):
 print("Stop message received")
 twist.linear.x = float(speed)
 twist.linear.y = 0
 twist.linear.z = 0
 twist.angular.x = 0
 twist.angular.y = 0
 twist.angular.z = 0
 publisher.publish(twist)

#we start broadcasting
def stop_broadcast():
 UDP_IP = "192.168.1.255"
 UDP_PORT = 5005
 MESSAGE = "STOP"
 for i in range(0,5):
  print("I AM SENDING THE SIGNAL!!!")
  sock = socket(AF_INET, SOCK_DGRAM) # UDP
  sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
  sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, UDP_PORT))

#function for front car
def front_car():
 time.sleep(1)
 move(0.2)
 time.sleep(3)
 move(0)
 stop_broadcast()
 print ("The mission completed! I have stopped and sent a signal.")

#function for back car
def back_car():
 s = socket(AF_INET, SOCK_DGRAM)
 s.bind(('', 5005))
 time.sleep(1)
 move(0.2)
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
        break
 rospy.spin()

#take argument and define the behaviour depending on the type of the car
if sys.argv[1] == "front":
 front_car()
if sys.argv[1] == "back":
 back_car()
