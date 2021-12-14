#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct
import signal
import logging
import argparse
import csv

shutdown = False

def _sigterm_handler(_signo, _stack_frame):
    global shutdown
    shutdown = True

def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class GulliViewPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    sent to the CSV file.
    """
    listen_tag_id = None  # Tag ID to listen for
    csv_writer = None  # Write your CSV here!

    def handle(self):
        # Receiving binary detection dafta from GulliView
        recv_buf = bytearray(self.request[0])
        logging.debug(f"Received {len(recv_buf)} bytes from {self.client_address}")

        # Fetch the type of message from the buffer data
        msg_type = unpack_data(recv_buf, start=0)
        sub_type = unpack_data(recv_buf, start=4)

        # Byte 8-12 is sequence number (unused)

        t1 = unpack_data(recv_buf, start=12)
        t2 = unpack_data(recv_buf, start=16)
        timestamp = ((t1 << 32) | t2) / 1000

        logging.debug(f"Message type: {msg_type}, subtype: {sub_type}, timestamp: {timestamp}")

        if msg_type == 1 and sub_type == 2:

            # The number of tags in this message
            length = unpack_data(recv_buf, start=28)

            logging.debug(f"Detections in packet: {length}")

            # Tag data from the GulliView server is placed in the buffer data
            # from the 32nd bit. A new tag is placed then placed every 16th bit
            for i in range(length):
                base = 32 + (16 * i)

                # Tag id, add 3 to offset subtraction done in GulliView (tags 0-3 are used for calibration)
                tag_id = unpack_data(recv_buf, start=base) + 3

                logging.debug(f"Detected tag id: {tag_id}")

                # If we aren't listening for all tags, and this is not the tag
                # we are listening for, skip it.
                if self.listen_tag_id != "all" and tag_id != self.listen_tag_id:
                    logging.debug("not interested, continuing...")
                    continue

                # X position of tag
                x = unpack_data(recv_buf, start=base + 4)

                # Y position of tag
                y = unpack_data(recv_buf, start=base + 8)

                # Camera capturing the tag
                c = unpack_data(recv_buf, start=base + 12)

                self.csv_writer.writerow([timestamp, tag_id, c, x, y])


if __name__ == "__main__":
    parser = argparse.ArgumentParser("gv_logger")

    parser.add_argument("-a", "--addr", help="The IP address to bind to", type=str, default="0.0.0.0")
    parser.add_argument("-p", "--port", help="The port to bind to", type=int, default=2121)
    parser.add_argument("-t", "--tag", help="Tag ID to filter by", default="all")
    parser.add_argument("-f", "--file", help="Filename to save to", default="gv.csv")
    args = parser.parse_args()
    
    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGINT, _sigterm_handler)

    # Set static variables on packet handler class to pass information to its instances
    GulliViewPacketHandler.listen_tag_id = args.tag

    print(f"Listening for UDP datagrams at {args.addr}:{args.port}, filtering by tag ID: {args.tag}")
    print(f"Data is being saved to {args.file}")
    with open(args.file, "w", newline='') as csvfile:
        GulliViewPacketHandler.csv_writer = csv.writer(csvfile, dialect='excel')
        GulliViewPacketHandler.csv_writer.writerow(["time", "tag", "camera", "x", "y"]) # Write header
        
        with UDPServer((args.addr, args.port), GulliViewPacketHandler) as server:
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.start()

            # Spin main thread until shutdown
            while not shutdown:
                signal.pause()

            # Shut down server when node shuts down
            print("Received signal, shutting down server")
            server.shutdown()
    print("Server shutdown, exiting")
    exit(0)

