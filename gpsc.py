#!/usr/bin/python

import socket

HOST = '127.0.0.1'
PORT = 2947

def tcp_client():
	client = socket.socket( socket.AF_INET, socket.SOCK_STREAM)
	client.connect(( HOST, PORT ))
	while True:
		print client.makefile().readline().rstrip()

if __name__ == '__main__':
    tcp_client()

