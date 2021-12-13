import json
from collections import namedtuple
import socket

Command = namedtuple('Command', ['desc', 'func'])


BROADCAST_IP = "192.168.1.255"
BROADCAST_PORT = 2424


def send_start():
    message = {'type': 'start'}

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.sendto(json.dumps(message).encode('utf-8'), (BROADCAST_IP, BROADCAST_PORT))
    print("Sent start message")


COMMANDS = {
    's': Command("Send start experiment command", send_start)
}


if __name__ == '__main__':
    print("Starting experiment controller")
    print("Available commands:")
    for key, command in COMMANDS.items():
        print(f"    {key}: {command.desc}")

    try:
        while True:
            command = input("Command > ")
            try:
                COMMANDS[command.lower()].func()
            except (KeyError, EOFError):
                print("Invalid command, try again")
    except KeyboardInterrupt:
        print("\nBye")
