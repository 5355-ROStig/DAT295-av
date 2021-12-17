#!/usr/bin/env python3
import argparse
import csv
import json
import signal
import struct
import threading
from socketserver import BaseRequestHandler, UDPServer
from typing import Union

from gullivutil import parse_packet


class Shutdown(Exception):
    """Raise to signal shutdown"""
    pass


def _sigterm_handler(_signo, _stack_frame):
    raise Shutdown


def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class GulliViewPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView position packets and log to CSV file

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    sent to the CSV file.
    """
    start_event: threading.Event = None
    listen_tag_id: Union[str, int] = None  # Tag ID to listen for, or 'all' (default)
    csv_writer = None  # Write your CSV here!

    def handle(self):
        if not self.start_event.is_set():
            # Skip messages before logging should start
            return

        # Receiving binary detection data from GulliView
        recv_buf = bytearray(self.request[0])
        packet = parse_packet(recv_buf)

        for det in packet.detections:
            # If we aren't listening for all tags, and this is not the tag
            # we are listening for, skip it.
            if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                continue

            self.csv_writer.writerow([packet.header.timestamp, det.tag_id, det.camera_id, det.x/1000, det.y/1000])


class ControllerPacketHandler(BaseRequestHandler):
    """
    Listen to experiment controller messages to start logging automatically.
    """
    start_event: threading.Event = None

    def handle(self):
        msg = json.loads(self.request[0])

        if msg['type'] == 'start' and not self.start_event.is_set():
            print("Received start from controller")
            self.start_event.set()


class IntersectionPacketHandler(BaseRequestHandler):
    """
    Listen to vehicle coordination messages to stop logging automatically.
    """
    start_event: threading.Event = None

    def handle(self):
        msg = json.loads(self.request[0])

        if not self.start_event.is_set():
            return

        if msg["MSGTYPE"] == "EXIT":
            # Stop logging
            print("Received coordination EXIT, stopping logging")
            self.start_event.clear()

            # TODO: rotate log file for next run


def run_server(server: UDPServer):
    thread_name = threading.current_thread().name
    server_address = ':'.join(map(str, server.server_address))
    print(f"Starting {thread_name} server on {server_address}")
    with server:
        server.serve_forever()


if __name__ == "__main__":
    parser = argparse.ArgumentParser("gv_logger")

    parser.add_argument("-a", "--addr", help="The IP address to bind to", type=str, default="0.0.0.0")
    parser.add_argument("-p", "--port", help="The position data port to bind to", type=int, default=2121)
    parser.add_argument("-c", "--coordport", help="The coordination message port to bind to", type=int, default=2323)
    parser.add_argument("-o", "--controlport", help="The control message port to bind to", type=int, default=2424)
    parser.add_argument("-t", "--tag", help="Tag ID to filter by", default="all")
    parser.add_argument("-f", "--file", help="Filename to save to", default="gv.csv")
    args = parser.parse_args()
    
    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGINT, _sigterm_handler)

    start_logging_event = threading.Event()

    print("Setting up control packet handler")
    ControllerPacketHandler.start_event = start_logging_event
    control_server = UDPServer((args.addr, args.controlport), ControllerPacketHandler)
    control_thread = threading.Thread(target=run_server, name='control', args=[control_server])

    print("Setting up coordination packet handler")
    IntersectionPacketHandler.start_event = start_logging_event
    coordination_server = UDPServer((args.addr, args.coordport), IntersectionPacketHandler)
    coordination_thread = threading.Thread(target=run_server, name='coordination', args=[coordination_server])

    print(f"Setting up datalogger, logging to {args.file}, filtering for tag: {args.tag}")
    csvfile = open(args.file, "w", newline='')
    GulliViewPacketHandler.start_event = start_logging_event
    GulliViewPacketHandler.listen_tag_id = args.tag
    GulliViewPacketHandler.csv_writer = csv.writer(csvfile, dialect='excel')
    GulliViewPacketHandler.csv_writer.writerow(["time", "tag", "camera", "x", "y"])  # Write header

    logging_server = UDPServer((args.addr, args.port), GulliViewPacketHandler)
    logging_thread = threading.Thread(target=run_server, name='logging', args=[logging_server])

    try:
        control_thread.start()
        coordination_thread.start()
        logging_thread.start()

        signal.pause()
    except Shutdown:
        pass
    finally:
        print("Received shutdown, cleaning up...")

        logging_server.shutdown()
        print("Logging server shutdown")

        control_server.shutdown()
        print("Control server shutdown")

        coordination_server.shutdown()
        print("Coordination server shutdown")

        csvfile.close()
        print("Logfile closed")

        print("Clean up complete, exiting")
