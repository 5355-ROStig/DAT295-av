import argparse
import json
import logging
from socketserver import BaseRequestHandler, UDPServer


class PacketLoggingHandler(BaseRequestHandler):
    logger: logging.Logger = None

    def handle(self) -> None:
        msg = json.loads(self.request[0])
        self.logger.info(f"{msg.pop('UID')} - {msg.pop('MSGTYPE')}{f' - Data: {msg}' if msg else ''}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Print coordination messages in real time")
    parser.add_argument('port', type=int, help="Port for UDP broadcast messages")

    args = parser.parse_args()
    host = '0.0.0.0'
    port = args.port

    log_format = "[%(asctime)s] - %(message)s"
    logging.basicConfig(level=logging.INFO, format=log_format)
    PacketLoggingHandler.logger = logging.getLogger()

    with UDPServer((host, port), PacketLoggingHandler) as server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.shutdown()
