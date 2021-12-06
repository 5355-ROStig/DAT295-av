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



cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist0 = Twist()
twist0.linear.x = 0
twist0.linear.y = 0
twist0.linear.z = 0
twist0.angular.x = 0
twist0.angular.y = 0
twist0.angular.z = 0

p1 = -2
p2 = -0.5
p3 = 0

pid = PID(p1, p2, p3, setpoint=70)
log = {"dist":list(),"speed":list(),"time":list()}

# C-c to save data
def signal_handler(sig, frame):
    with open('lidar.log','w') as f:
        json.dump(log,f)
    print("Saving")
    sys.exit(0)

def laser_callback_pid(data):
    filtered = []
    for i in data.ranges:
        if i > 0.002:
            filtered.append(i)
    min_distance = min(filtered) * 100  # convert to cm
    print(str(min_distance))
    control = pid(min_distance)
    twist0.linear.x = control / 100
    print("Dist: " + str(min_distance) + "cm")
    print("Speed: " + str(twist0.linear.x) + "m/s")
    log['dist'].append(min_distance)
    log['speed'].append(twist0.linear.x)
    log['time'].append(time.time())

    if(twist0.linear.x>0.8):
        twist0.linear.x = 0.8
    if(twist0.linear.x<-0.8):
        twist0.linear.x = -0.8
    cmd_vel_publisher.publish(twist0)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('IR_receiver', anonymous=True)

    #rospy.Subscriber("/IR", IR, callback)
    rospy.Subscriber("/scan_filtered", LaserScan, laser_callback_pid)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    time.sleep(1)
    signal.signal(signal.SIGINT, signal_handler)
    listener()
