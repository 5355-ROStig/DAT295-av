#!/usr/bin/env python3
import json
import threading
from socketserver import BaseRequestHandler, UDPServer

import rospy
from std_msgs.msg import Empty


START_TOPIC = "/experiment_start"


class ExperimentControlPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    unpacked and sent on the ROS topic that ``cls.Publisher`` is publishing on.
    """
    start_publisher = None  # Static reference to ROS topic publisher

    def handle(self):
        msg = json.loads(self.request[0])
        rospy.loginfo(f"Handling received controller message: {msg}")

        if msg['type'] == 'START':
            rospy.loginfo("Received experiment start message")
            self.start_publisher.publish(Empty())


if __name__ == "__main__":
    rospy.init_node('control_client', anonymous=True)
    rospy.loginfo(f"Started experiment control node")

    topic = rospy.names.canonicalize_name(START_TOPIC)
    rospy.loginfo(f"Setting up publisher on {topic}")

    host = rospy.get_param("~host", default="0.0.0.0")
    port = rospy.get_param("~port", default=2424)

    # Set static variables on packet handler class to pass information to its instances
    ExperimentControlPacketHandler.start_publisher = rospy.Publisher(topic, Empty, queue_size=1)

    rospy.loginfo(f"Listening for experiment control messages on {host}:{port} (UDP)")
    with UDPServer((host, port), ExperimentControlPacketHandler) as server:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        # Spin main thread (the ROS node) until shutdown
        rospy.spin()

        # Shut down server when node shuts down
        rospy.loginfo("Node received shutdown signal, shutting down server")
        server.shutdown()
        rospy.loginfo("Server shutdown, exiting")
