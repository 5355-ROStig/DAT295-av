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

pid = PID(-2.5, -0.8, 0, setpoint=50)
log = list()

# C-c to save data
def signal_handler(sig, frame):
    with open('./log_for_PID','w') as f:
        json.dump(log,f)
    print("Saving")
    sys.exit(0)

def callback(data):
    #print("aaaa")
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.IR_back_right)

    maxspeed = 0.4
    if (maxspeed > 0.5):
        maxspeed = 0.5
    if (data.IR_front_right < 50 or data.IR_front_left < 50):
        if (twist0.linear.x - 0.01 <= 0):
            twist0.linear.x = 0
        else:
            twist0.linear.x -= 0.01
    else:
        if (twist0.linear.x < maxspeed):
            twist0.linear.x += 0.01
        else:
            twist0.linear.x -= 0.01
    cmd_vel_publisher.publish(twist0)


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
    log.append({'dist':min_distance,'speed':twist0.linear.x,'time':time.time()})
    if(twist0.linear.x>0.5):
        twist0.linear.x = 0.5
    if(twist0.linear.x<-0.5):
        twist0.linear.x = -0.5
    cmd_vel_publisher.publish(twist0)


def laser_callback(data):
    filtered = []
    for i in data.ranges:
        if i > 0.002:
            filtered.append(i)
    min_distance = min(filtered) * 100  # convert to cm
    print(str(min_distance))
    maxspeed = min_distance / 50
    if (maxspeed > 0.6):
        maxspeed = 0.6
    if (min_distance < 90 or min_distance < 90):
        if (twist0.linear.x - 0.05 <= 0):
            twist0.linear.x = 0
        else:
            twist0.linear.x -= 0.1
    else:
        if (twist0.linear.x < maxspeed):
            twist0.linear.x += maxspeed / 85
        else:
            twist0.linear.x -= 0.10
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
