#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rospy
from std_msgs.msg import Header

from gv_client.msg import GulliViewPosition
from gullivutil import parse_packet

GV_POSITION_TOPIC = "gv_positions"


def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class GulliViewPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    unpacked and sent on the ROS topic that ``cls.Publisher`` is publishing on.
    """
    publisher = None  # Static reference to ROS topic publisher
    listen_tag_id = None  # Tag ID to listen for

    def handle(self):
        # Receiving binary detection data from GulliView
        recv_buf = bytearray(self.request[0])
        packet = parse_packet(recv_buf)

        for det in packet.detections:
            # If we aren't listening for all tags, and this is not the tag
            # we are listening for, skip it.
            if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                continue

            header = Header()
            header.stamp = rospy.Time.from_sec(packet.header.timestamp)
            msg = GulliViewPosition(header=header, x=det.x, y=det.y, tagId=det.tag_id, cameraId=det.camera_id)

            self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node('gulliview', anonymous=True)
    rospy.loginfo(f"Started gulliview node")

    topic = rospy.names.canonicalize_name(GV_POSITION_TOPIC)
    rospy.loginfo(f"Setting up publisher on {topic}")

    host = rospy.get_param("~host", default="0.0.0.0")
    port = rospy.get_param("~port", default=2121)
    listen_tag_id = rospy.get_param("/tag_id", default="all")

    # Set static variables on packet handler class to pass information to its instances
    GulliViewPacketHandler.publisher = rospy.Publisher(topic, GulliViewPosition, queue_size=10)
    GulliViewPacketHandler.listen_tag_id = listen_tag_id

    rospy.loginfo(f"Starting UDP server on {host}:{port}, listening for tag ID: {listen_tag_id}")
    with UDPServer((host, port), GulliViewPacketHandler) as server:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        # Spin main thread (the ROS node) until shutdown
        rospy.spin()

        # Shut down server when node shuts down
        rospy.loginfo("Node received shutdown signal, shutting down server")
        server.shutdown()
        rospy.loginfo("Server shutdown, exiting")
