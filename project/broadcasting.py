#!/usr/bin/python3

from socket import *
import time
UDP_IP = "192.168.1.255"
UDP_PORT = 5005
MESSAGE = "STOP"
while True:
 print("I AM SENDING THE SIGNAL!!!")
 sock = socket(AF_INET, SOCK_DGRAM) # UDP
 sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
 sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, UDP_PORT))
 time.sleep(1)

