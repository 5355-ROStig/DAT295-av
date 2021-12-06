#!/usr/bin/python3
import rospy
from roswifibot.msg import IR
from geometry_msgs.msg import Twist
import time
from sensor_msgs.msg import LaserScan
from simple_pid import PID
import json
import signal
import sys
import threading
from socket import *

g_speed = 0

log = {"dist":list(),"speed":list(),"time":list()}

rospy.init_node("useCase4Back")
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

# C-c to save data
def signal_handler(sig, frame):
    with open('lidar.log','w') as f:
        json.dump(log,f)
    print("Saving")
    sys.exit(0)


def listen_for_speed():
 s = socket(AF_INET, SOCK_DGRAM)
 global g_speed
 s.bind(('', 5005))
 while True:
    print('Ivan is waiting your order!')
    m = s.recvfrom(5005)
    m = m[0].decode("utf-8")
    print (m)
    m = json.loads(m)
    print ("Speed received: ", str(m['speed']))
    g_speed = m['speed']


def start():
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print ("Inside the loop", g_speed)
            twist.linear.x = float(g_speed)
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            publisher.publish(twist)
            rate.sleep()


def listener():

    #rospy.Subscriber("/IR", IR, callback)
    rospy.Subscriber("/scan_filtered", LaserScan, laser_callback_pid)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





if __name__ == '__main__':
    time.sleep(1)
    signal.signal(signal.SIGINT, signal_handler)
    # add record / log
    # listen for speed - run in parallel
    # adjust speed after received speed - run in parallel
    t = threading.Thread(target=listen_for_speed)
    t.start()

    t2 = threading.Thread(target=start)
    t2.start()
    # listener()
