#!/usr/bin/python

import logging, sys, time, json
import socket, threading
import nazagpsdecoder

BIND_IP = '127.0.0.1'
BIND_PORT = 2947

message_lock = threading.Lock()
message = {}
message["gps"] = {}
message["com"] = {}
threads = []
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)


def tvp_message():
	global message, message_lock

	message_lock.acquire()
	try:
		tvp = {
			"class": "TPV",
			"device": "naza",
			"time": "2005-06-08T10:34:48.283Z",
			"lat": message["gps"]["decoded"]["lat"],
			"lon": message["gps"]["decoded"]["lon"],
			"alt": message["gps"]["decoded"]["gpsAlt"],
			"track": message["com"]["decoded"]["heading"],
			"speed": message["gps"]["decoded"]["spd"],
			"mode": message["gps"]["decoded"]["fix_type"],
			"satelites": message["gps"]["decoded"]["satelites"],
		}
	except:
		tvp = {}
	message_lock.release()

	return tvp


def gps_reader():
	global message, message_lock, process
	decoder = nazagpsdecoder.NazaGpsDecoder()

	while True:
		gmsg = decoder.readMessage("gps")
		message_lock.acquire()
		message["gps"] = gmsg
		message_lock.release()

		cmsg = decoder.readMessage("com")
		message_lock.acquire()
		message["com"] = cmsg
		message_lock.release()
	
		time.sleep(1)


#http://www.catb.org/gpsd/gpsd_json.html
#https://github.com/aircrack-ng/aircrack-ng/blob/master/src/airodump-ng.c
def handle_client(client_socket, addr):
	logging.info("Accepted connection from: %s:%d" %(addr[0], addr[1]))
	try:
		while True:
			client_socket.send(json.dumps(tvp_message())+"\n")
			time.sleep(1)
	except Exception as e:
		#logging.info(e)
		pass
	client_socket.close()
	logging.info("Closed connection from: %s:%d" %(addr[0], addr[1]))


def tcp_server():
	global threads
	server = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
	server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server.bind(( BIND_IP, BIND_PORT))
	server.listen(5)
	logging.info("Listening on %s:%d" % (BIND_IP, BIND_PORT))

	while True:
	        client, addr = server.accept()
	        client_handler = threading.Thread(target=handle_client, args=(client,addr))
	        client_handler.start()
		threads.append(client_handler)


def gpsd():
	global threads
	try:
		t_gps = threading.Thread(target = gps_reader)
		t_gps.setDaemon(True)
		t_gps.start()

		t_server = threading.Thread(target = tcp_server)
		t_server.setDaemon(True)
		t_server.start()

		while True:
			logging.info(json.dumps(tvp_message()))
			for t in threads:
				if t.is_alive() == False:
					t.join()
					threads.remove(t)
			#logging.info(threads)
			time.sleep(1)

		t_gps.join()
		t_server.join()

	except (KeyboardInterrupt, SystemExit):
		sys.exit()


if __name__ == '__main__':
	gpsd()
