#!/bin/env python3
import threading
import typing
from collections import namedtuple
from typing import Optional, Callable
from socketserver import UDPServer, BaseRequestHandler
import socket
import json

import rospy

from gv_client.msg import GulliViewPosition
from mapdata.srv import GetIntersection
from mapdata.msg import RoadSection
from std_msgs.msg import Empty
from coordination_strategies import COORDINATION_STRATEGIES

if typing.TYPE_CHECKING:
    from coordination.src.coordination_strategies.strategy import CoordinationStrategy

Position = namedtuple('Position', ['x', 'y'])
BROADCAST_IP = "192.168.1.255"


class CoordinationNode:
    def __init__(self, port):
        rospy.init_node('coordination_node', anonymous=True)
        rospy.loginfo("Starting intersection coordination protocol node")

        self.pos: Optional[Position] = None

        self.port = port
        # Global tag id parameter
        self.tag_id = rospy.get_param("/tag_id")

        # Flags for coordination stages
        self.coordinator_lock = threading.Lock()
        with self.coordinator_lock:
            self.enter_rcvd = False
            self.other_croad = None
            self.ack_rcvd = False
            self.exit_rcvd = False
            self.exit_topic_rcvd = False  # /exit ros topic
            self.stopped_topic_rcvd = False  # /stopped ros topic

        scenario_param = rospy.get_param('~scenario')
        rospy.loginfo(f"Loading mission for requested scenario '{scenario_param}'")

        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)

        # Wait for initial position data
        try:
            rospy.loginfo("Awaiting initial position data")
            self._await(self._initial_position_received, timeout=30.0)
        except TimeoutError as e:
            rospy.logerr("Timeout while waiting for initial position")
            raise e
        rospy.loginfo(f"Initial position received: ({self.pos.x}, {self.pos.y})")

        # Query for environment/map data
        rospy.loginfo("Waiting for map data service")
        rospy.wait_for_service('intersection_data')
        get_road_data = rospy.ServiceProxy('intersection_data', GetIntersection)
        try:
            rospy.loginfo("Querying map data service...")
            self.road_data = get_road_data()
        except rospy.ServiceException as e:
            rospy.logerr("Error while fetching road data from map data server", e)
            rospy.signal_shutdown()
        rospy.loginfo("Retrieved map data")

        rospy.loginfo("Determining mission parameters")
        # Figure out which road we are starting from
        self.start_road = self._determine_starting_road()

        # Figure out the other mission parameters
        destinations = {
            'N': self.road_data.south,
            'S': self.road_data.north,
            'E': self.road_data.west,
            'W': self.road_data.east
        }

        self.destination_road = destinations[self.start_road.name]
        self.start_line = self._find_stopline(self.start_road)
        self.stop_line = self._find_stopline(self.destination_road)
        self.strategy: CoordinationStrategy = COORDINATION_STRATEGIES[self.start_road.priority_sign](self)

        rospy.Subscriber("stopped", Empty, self._receive_stopped)
        rospy.Subscriber("exit", Empty, self._receive_exit)

    @staticmethod
    def _await(condition: Callable[[], bool], rate: int = 2, timeout: float = 10.0):
        """Block until a given condition is True.

        Raises TimeoutError if condition does not become true before timeout.

        :param condition: A callable taking no arguments and returning a falsy value
                          while the condition is not met, and a truthy value once it is.
        :param rate: The frequency in Hz at which the condition should be polled
        :param timeout: Number of seconds before action times out.
        """
        rate = rospy.Rate(rate)
        timeout = rospy.Duration(secs=timeout)

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < timeout:
            if condition():
                return
            rate.sleep()
        raise TimeoutError()

    def _initial_position_received(self) -> bool:
        return self.pos is not None

    def _position_cb(self, position_msg):
        self.pos = Position(position_msg.x, position_msg.y)

    def _determine_starting_road(self):
        for road_section in (self.road_data.north, self.road_data.south, self.road_data.east, self.road_data.west):
            if road_section.name == 'N':
                min_x = road_section.right.x
                max_x = road_section.left.x
                max_y = max(road_section.left.y, road_section.right.y)
                min_y = max_y - road_section.length
            elif road_section.name == 'S':
                min_x = road_section.left.x
                max_x = road_section.right.x
                min_y = min(road_section.left.y, road_section.right.y)
                max_y = min_y + road_section.length
            elif road_section.name == 'E':
                min_y = road_section.right.y
                max_y = road_section.left.y
                min_x = min(road_section.left.x, road_section.right.x)
                max_x = min_x + road_section.length
            elif road_section.name == 'W':
                min_y = road_section.left.y
                max_y = road_section.right.y
                max_x = max(road_section.left.x, road_section.right.x)
                min_x = max_x - road_section.length

            if (min_x <= self.pos.x <= max_x) and (min_y <= self.pos.y <= max_y):
                return road_section
        raise RuntimeError("Unable to determine starting road! "
                           "Make sure robot is positioned properly")

    @staticmethod
    def _find_stopline(road_section):
        if road_section.name == 'N':
            return min(road_section.left.y, road_section.right.y) - road_section.stopline_offset
        elif road_section.name == 'S':
            return min(road_section.left.y, road_section.right.y) + road_section.stopline_offset
        elif road_section.name == 'E':
            return min(road_section.left.x, road_section.right.x) + road_section.stopline_offset
        elif road_section.name == 'W':
            return min(road_section.left.x, road_section.right.x) - road_section.stopline_offset

    def execute_protocol(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Init the publisher here so subscribers have enough time to register it (?)
        go_pub = rospy.Publisher("/go", Empty, queue_size=1)

        enter_msg = {
            "UID": self.tag_id,
            "MSGTYPE": "ENTER",
            "CROAD": self.start_road.name  # CROAD=current road
        }

        ack_msg = {
            "UID": self.tag_id,
            "MSGTYPE": "ACK"
        }

        exit_msg = {
            "UID": self.tag_id,
            "MSGTYPE": "EXIT"
        }

        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Waiting for ENTER...")
        while not self.enter_rcvd and not rospy.is_shutdown():
            # Send data as json
            data = bytes(json.dumps(enter_msg), "utf-8")

            s.sendto(data, (BROADCAST_IP, self.port))
            rate.sleep()

        rospy.loginfo("ENTER received, waiting for ACK...")

        # ACK
        data = bytes(json.dumps(ack_msg), "utf-8")
        s.sendto(data, (BROADCAST_IP, self.port))
        while not self.ack_rcvd and not rospy.is_shutdown():
            # We still send ENTER in case the other bot is still waiting for ours
            data = bytes(json.dumps(enter_msg), "utf-8")
            s.sendto(data, (BROADCAST_IP, self.port))

            # ACK
            data = bytes(json.dumps(ack_msg), "utf-8")
            s.sendto(data, (BROADCAST_IP, self.port))

            rate.sleep()

        rospy.loginfo("ACK received, resolving priority...")

        while not (self.strategy.has_priority() or self.exit_rcvd) and not rospy.is_shutdown():
            # ACK
            data = bytes(json.dumps(ack_msg), "utf-8")
            s.sendto(data, (BROADCAST_IP, self.port))
            rate.sleep()

        rospy.loginfo("Sending /go to mission planner")
        go_pub.publish(Empty())

        rospy.loginfo("Sending ACKs until mission planner says I've crossed")
        while not self.exit_topic_rcvd and not rospy.is_shutdown():
            # ACK
            data = bytes(json.dumps(ack_msg), "utf-8")
            s.sendto(data, (BROADCAST_IP, self.port))
            rate.sleep()

        rospy.loginfo("Spamming EXIT for all eternity :)")
        while not rospy.is_shutdown():
            # EXIT
            data = bytes(json.dumps(exit_msg), "utf-8")
            s.sendto(data, (BROADCAST_IP, self.port))
            rate.sleep()

    def _receive_stopped(self, _):
        rospy.loginfo(f"/stopped received from mission planner ({self.stopped_topic_rcvd})")
        self.stopped_topic_rcvd = True

    def _receive_exit(self, _):
        rospy.loginfo("/exit received from mission planner")
        self.exit_topic_rcvd = True


class IntersectionPacketHandler(BaseRequestHandler):
    coordinator_node = None  # Static reference to ROS topic publisher

    def handle(self):
        msg = json.loads(self.request[0])
        
        if msg['UID'] == self.coordinator_node.tag_id:  
            # Ignore our own broadcasts
            return
        
        # rospy.loginfo(f"Received packet: {msg} of type {msg['MSGTYPE']}")

        with self.coordinator_node.coordinator_lock:
            if msg["MSGTYPE"] == "ENTER":
                self.coordinator_node.enter_rcvd = True
                self.coordinator_node.other_croad = msg['CROAD']
            elif msg["MSGTYPE"] == "ACK":
                self.coordinator_node.ack_rcvd = True
            elif msg["MSGTYPE"] == "EXIT":
                self.coordinator_node.exit_rcvd = True


if __name__ == '__main__':
    host = "0.0.0.0"
    port = rospy.get_param("~port", default=2323)

    coordination_node = CoordinationNode(port)

    IntersectionPacketHandler.coordinator_node = coordination_node

    rospy.loginfo(f"Starting UDP server on {host}:{port}")
    with UDPServer((host, port), IntersectionPacketHandler) as server:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        # Begin executing protocol
        coordination_node.execute_protocol()

        # Shut down server when node shuts down
        rospy.loginfo("Node received shutdown signal, shutting down server")
        server.shutdown()
        rospy.loginfo("Server shutdown, exiting")
