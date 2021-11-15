#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rospy

from gv_client.msg import GulliViewPosition


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
        # Receiving binary detection dafta from GulliView
        recv_buf = bytearray(self.request[0])
        rospy.loginfo(f"Received {len(recv_buf)} bytes from {self.client_address}")

        # Fetch the type of message from the buffer data
        msg_type = unpack_data(recv_buf, start=0)
        sub_type = unpack_data(recv_buf, start=4)

        if msg_type == 1 and sub_type == 2:

            # The number of tags in this message
            length = unpack_data(recv_buf, start=28)

            # Tag data from the GulliView server is placed in the buffer data
            # from the 32nd bit. A new tag is placed then placed every 16th bit
            for i in range(length):
                base = 32 + (16 * i)

                # Tag id, add 3 to offset subtraction done in GulliView (tags 0-3 are used for calibration)
                tag_id = unpack_data(recv_buf, start=base) + 3

                # If we aren't listening for all tags, and this is not the tag
                # we are listening for, skip it.
                if self.listen_tag_id != "all" and tag_id != self.listen_tag_id:
                    continue

                # X position of tag
                x = unpack_data(recv_buf, start=base + 4)

                # Y position of tag
                y = unpack_data(recv_buf, start=base + 8)

                # Camera capturing the tag
                c = unpack_data(recv_buf, start=base + 12)

                msg = GulliViewPosition(x=x, y=y, tagId=tag_id, cameraId=c)

                self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node('gulliview', anonymous=True)
    rospy.loginfo(f"Started gulliview node")

    topic = rospy.names.canonicalize_name(GV_POSITION_TOPIC)
    rospy.loginfo(f"Setting up publisher on {topic}")

    host = rospy.get_param("~host", default="0.0.0.0")
    port = rospy.get_param("~port", default=2121)
    listen_tag_id = rospy.get_param("~tag_id", default="all")

    # Set static variables on packet handler class to pass information to its instances
    GulliViewPacketHandler.publisher = rospy.Publisher(topic, GulliViewPosition, queue_size=10)
    GulliViewPacketHandler.tag_id = listen_tag_id

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
