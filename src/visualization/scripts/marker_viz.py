#!/usr/bin/python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from gv_client.msg import GulliViewPosition
from zone_marker import ZoneMarker

class Visualizer():
    def __init__(self):
        rospy.init_node('visualizer')

        # Create markers for each wifibot
        self.bot_markers = {
            # 4: self.init_marker(4),
            5: self.init_marker(5),
            6: self.init_marker(6)
        }


        rospy.loginfo(f"Visualizer started, looking for tags: {list(self.bot_markers.keys())}")

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        rospy.Subscriber("/gv_positions", GulliViewPosition, self.bot_state_handler)

        zone_pub = rospy.Publisher('zone_marker', Marker, queue_size=1, latch=True)
        # Show critical section in a nice pink color
        section_marker = ZoneMarker(zone_id = 0, zone_type = 'critical_section',
                                    origin = (1795, 6106), width = 800,
                                    height = 800, color = (255, 85, 127), y_mirroring = 0)
        zone_pub.publish(section_marker.marker)

    def bot_state_handler(self, gv_position):
        tag_id = gv_position.tagId

        if tag_id not in self.bot_markers:
            rospy.logerr(f"Detected unknown tagId: {tag_id}")
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        # This frame name comes from mapdata package I think
        t.header.frame_id = "map"
        t.child_frame_id = self.bot_markers[tag_id].header.frame_id

        t.transform.translation.x = gv_position.x
        t.transform.translation.y = gv_position.y
        t.transform.translation.z = 0

        # Must be set even though we don't care about rotation
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        self.marker_pub.publish(self.bot_markers[tag_id])
        tf2_ros.TransformBroadcaster().sendTransform(t)



    def init_marker(self, bot_id):
        m = Marker()

        # ns+id and frame_id needs to be unique
        m.header.frame_id = "bot_frame" + str(bot_id)
        m.ns = "bot"
        m.id = bot_id

        m.header.stamp = rospy.Time(0)
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Position is always at origin of the bot's frame
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 100  # Place above map

        # Orientation is always identity
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1

        # Size of marker
        m.scale.x = 200
        m.scale.y = 200
        m.scale.z = 200

        # RGBA colors
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0

        # Persistent marker
        m.lifetime = rospy.Duration(0)

        return m


if __name__ == "__main__":
    Visualizer()
    rospy.spin()