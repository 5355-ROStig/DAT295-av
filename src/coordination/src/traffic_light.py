from time import sleep
import json
import threading
import socket
import argparse
import signal

from socketserver import UDPServer, BaseRequestHandler

BROADCAST_IP = "192.168.1.255"
shutdown = False


def _sigterm_handler(_signo, _stack_frame):
    global shutdown
    shutdown = True



class TrafficLight:
    def __init__(self):
        self.uid = 0
        # Flags for coordination stages
        self.lock = threading.Lock()
        with self.lock:
            self.first_enter_rcvd = False
            self.snd_enter_rcvd = False
            self.first_ack_rcvd = False
            self.snd_ack_rcvd = False

        # Socket for sending V2V/V2I packages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def broadcast_msg(self, msg):
        data = bytes(json.dumps(msg), "utf-8")
        self.sock.sendto(data, (BROADCAST_IP, self.port))

    def execute_protocol(self):
        rate = 10  # Hz
        print("Waiting for ENTER from both cars")
        while not shutdown and not self.first_enter_rcvd and not self.snd_enter_rcvd:
            # Do nothing when waiting for
            sleep(1 / rate)


class TrafficPacketHandler(BaseRequestHandler):
    traffic_light = None

    def handle(self):
        msg = json.loads(self.request[0])

        if msg['UID'] == self.traffic_light.uid:
            # Ignore our own broadcasts
            return

        with self.traffic_light.lock:
            if msg['MSGTYPE'] == 'ENTER':
                if not self.traffic_light.first_enter_rcvd:
                    self.traffic_light.first_enter_rcvd = True
                else:
                    self.traffic_light.snd_enter_rcvd = True
            elif msg['MSGTYPE'] == 'ACK':
                if not self.traffic_light.first_ack_rcvd:
                    self.traffic_light.first_ack_rcvd = True
                else:
                    self.traffic_light.snd_ack_rcvd = True
            elif msg['MSGTYPE'] == 'EXIT':
                ...
            else:
                print(f"Traffic light packet handler received unexpected message type: {msg['MSGTYPE']}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser("traffic_light")
    parser.add_argument("-a", "--addr", help="The IP address bind to", type=str, default="0.0.0.0")
    parser.add_argument("-p", "--port", help="The port to bind to", type=int, default=2323)
    args = parser.parse_args()

    host = args.host
    port = args.port

    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGINT, _sigterm_handler)

    tl = TrafficLight()
    TrafficPacketHandler.traffic_light = tl

    print(f'Starting UDP server on {host}:{port}')
    with UDPServer((host, port), TrafficPacketHandler) as server:
        # Start new thread for UDP server
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        tl.execute_protocol()

        # Shut down server when node shuts down
        print('Shutting down server')
        server.shutdown()
    print('Server shutdown, exiting')
    exit(0)
