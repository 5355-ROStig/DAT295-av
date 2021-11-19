#!/usr/bin/python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from gv_client.msg import GulliViewPosition

class GulliViewMarker():
    def __init__(self):
        rospy.init_node('visualizer')

        self.marker = self.init_marker()
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        rospy.Subscriber("/gv_positions", GulliViewPosition, self.marker_state_handler)


    def marker_state_handler(self, gv_position):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = "map"
        t.child_frame_id = self.marker.header.frame_id
        t.transform.translation.x = gv_position.x
        t.transform.translation.y = gv_position.y
        t.transform.translation.z = 0

        # Must be set even though we don't care about rotation
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.marker_pub.publish(self.marker)
        tf2_ros.TransformBroadcaster().sendTransform(t)


    def init_marker(self):
        m = Marker()

        m.header.frame_id = "marker_frame"
        m.header.stamp = rospy.Time(0)

        m.ns = "Test marker"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Position is always at origin of the frame
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 20  # We're only working in 2D
        # Orientation is always identity
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.scale.x = 100
        m.scale.y = 100
        m.scale.z = 100
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0

        # Persist
        m.lifetime = rospy.Duration(0)
        return m


if __name__ == "__main__":
    GulliViewMarker()
    rospy.spin()