#!/usr/bin/python3

from socket import *
import time
#UDP_IP = "127.0.0.1"
UDP_IP = "192.168.1.255"
UDP_PORT = 5005
MESSAGE = "START"
#while True:
for i in range(0,3):
 print("Start the party now!")
 sock = socket(AF_INET, SOCK_DGRAM) # UDP
 sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
 sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, UDP_PORT))
 time.sleep(1)
 

