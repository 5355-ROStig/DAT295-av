from time import sleep
import json
import threading
import socket
import argparse

from socketserver import UDPServer, BaseRequestHandler

BROADCAST_IP = "192.168.1.255"


class Car:
    """
    Maintains a set of flags for each car communicating with the traffic light.
    """

    def __init__(self):
        self.uid = None
        self.ip = None
        self.enter_rcvd = False
        self.ack_rcvd = False
        self.exit_rcvd = False

    def __hash__(self):
        return hash(self.uid)


class TrafficLight:
    def __init__(self, port):
        self.uid = 0
        # Flags for coordination stages
        self.lock = threading.Lock()
        with self.lock:
            # The cars we expect to send an ENTER msg to the traffic light
            self.first_car = Car()
            self.snd_car = Car()

        # Socket for sending V2V/V2I packages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.port = port

        # Message types
        self.ack_msg = {"UID": self.uid, "MSGTYPE": "ACK"}
        self.exit_msg = {"UID": self.uid, "MSGTYPE": "EXIT"}
        self.sched_msg = lambda schedule: {"UID": self.uid, "MSGTYPE": "SCHED", "SCHED_LIST": schedule}

    def broadcast_msg(self, msg):
        data = json.dumps(msg).encode("utf-8")
        self.sock.sendto(data, (BROADCAST_IP, self.port))

    def unicast_msg(self, msg, ip):
        data = json.dumps(msg).encode("utf-8")
        self.sock.sendto(data, (ip, self.port))

    def execute_protocol(self):
        rate = 10  # Hz
        print("Waiting for ENTER from both cars")
        while not (self.first_car.enter_rcvd and self.snd_car.enter_rcvd):
            # Do nothing when waiting for ENTER
            sleep(1 / rate)

        # Simple strategy for now, just choose whoever sent ENTER first
        schedule = [self.first_car, self.snd_car]
        # Schedule needs to be json serializable, quick fix: use another list ¯\_(ツ)_/¯
        uid_schedule = [self.first_car.uid, self.snd_car.uid]
        print(f"Waiting for ACK from both cars, spamming schedule: {uid_schedule}")

        self.broadcast_msg(self.sched_msg(uid_schedule))
        while not (self.first_car.ack_rcvd and self.snd_car.ack_rcvd):
            self.broadcast_msg(self.sched_msg(uid_schedule))
            sleep(1 / rate)

        print(f"Waiting for EXIT from the first scheduled car, uid {schedule[0].uid}")
        while not schedule[0].exit_rcvd:
            sleep(1 / rate)

        print(f"Waiting for ACK2 and EXIT2")
        self.unicast_msg(self.ack_msg, schedule[0].ip)  # ACK1
        self.unicast_msg(self.sched_msg(uid_schedule[1:]), schedule[1].ip)  # SCHED2

        while not (schedule[1].ack_rcvd and schedule[1].exit_rcvd):
            self.unicast_msg(self.ack_msg, schedule[0].ip)  # ACK1
        self.unicast_msg(self.sched_msg(uid_schedule[1:]), schedule[1].ip)  # SCHED2

        print(f"Broadcasting ACKs forever")
        while True:
            self.broadcast_msg(self.ack_msg)


class TrafficPacketHandler(BaseRequestHandler):
    traffic_light = None

    def handle(self):
        msg = json.loads(self.request[0])

        if msg['UID'] == self.traffic_light.uid:
            # Ignore our own broadcasts
            return

        # print(f"Received msg {msg} from {self.client_address[0]}")

        sender_uid = msg['UID']
        sender_ip = self.client_address[0]

        with self.traffic_light.lock:
            if msg['MSGTYPE'] == 'ENTER':
                if not self.traffic_light.first_car.enter_rcvd:
                    self.traffic_light.first_car.uid = sender_uid
                    self.traffic_light.first_car.ip = sender_ip
                    self.traffic_light.first_car.enter_rcvd = True

                elif not self.traffic_light.snd_car.enter_rcvd and sender_uid != self.traffic_light.first_car.uid:
                    self.traffic_light.snd_car.uid = sender_uid
                    self.traffic_light.snd_car.ip = sender_ip
                    self.traffic_light.snd_car.enter_rcvd = True

            from_first = sender_uid == self.traffic_light.first_car.uid

            if msg['MSGTYPE'] == 'ACK':
                if from_first:
                    self.traffic_light.first_car.ack_rcvd = True
                else:
                    self.traffic_light.snd_car.ack_rcvd = True
            if msg['MSGTYPE'] == 'EXIT':
                if from_first:
                    self.traffic_light.first_car.exit_rcvd = True
                else:
                    self.traffic_light.snd_car.exit_rcvd = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser("traffic_light")
    parser.add_argument("-a", "--addr", help="The IP address bind to", type=str, default="0.0.0.0")
    parser.add_argument("-p", "--port", help="The port to bind to", type=int, default=2323)
    args = parser.parse_args()

    tl = TrafficLight(args.port)
    TrafficPacketHandler.traffic_light = tl

    print(f'Starting UDP server on {args.addr}:{args.port}')
    with UDPServer((args.addr, args.port), TrafficPacketHandler) as server:
        # Start new thread for UDP server
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        try:
            tl.execute_protocol()
        except KeyboardInterrupt:
            pass

        print('\nShutting down server')
        server.shutdown()

    print('Server shutdown, exiting')
