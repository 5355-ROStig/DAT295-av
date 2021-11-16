#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

def main():
    rospy.init_node('visualizer')

    m = init_marker()

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10, latch=True)
    marker_pub.publish(m)
    rospy.spin()


def init_marker():
    m = Marker()

    m.header.frame_id = "map"
    m.header.stamp = rospy.Time(0)

    m.ns = "Test marker"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD

    # Position is always at origin of the frame
    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0  # We're only working in 2D
    # Orientation is always identity
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = 1
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0

    # Persist
    m.lifetime = rospy.Duration(0)
    return m


if __name__ == "__main__":
    main()