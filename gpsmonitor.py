#!/usr/bin/python

import logging, sys, time
import threading
import nazagpsdecoder

message_lock = threading.Lock()
message = {}
message["gps"] = {}
message["com"] = {}
message["ver"] = {}

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)

def data_reader():
	global message, message_lock
	try:
		decoder = nazagpsdecoder.NazaGpsDecoder()
		while True:
			msg = decoder.readRawMessage()
			msg["decoded"] = decoder.decodeMessage(msg)
			message_lock.acquire()
			message[msg["type"]] = msg
			message_lock.release()

	except Exception as e:
		#logging.warn("stopping data_reader")
		pass
		

def data_printer():
	global message, message_lock
	fmt = "%6s %20s %32s %4s %-10s %-10s %-13s %-16s"
	columns = ("seq", "datetime", "bintime", "sats", "lon", "lat", "heading", "spd")
	logging.info(fmt % columns)
	try:
		while True:
			message_lock.acquire()
			if message["gps"]!={} and message["com"]!={} and message["ver"]!={}:
				logging.info(fmt % (
					message["gps"]["decoded"]["seq"], 
					message["gps"]["decoded"]["datetime"], 
					bin(message["gps"]["decoded"]["time"]),
					message["gps"]["decoded"]["satelites"], 
					message["gps"]["decoded"]["lon"], 
					message["gps"]["decoded"]["lat"], 
					message["com"]["decoded"]["heading"], 
					message["gps"]["decoded"]["spd"], 
				) )
			message_lock.release()
			time.sleep(1)
	except Exception as e:
		logging.warn(e)
		#logging.warn("stopping data_printer")
		pass

#main
try:
	t1 = threading.Thread(target = data_reader)
	t1.setDaemon(True)
	t1.start()
	t2 = threading.Thread(target = data_printer)
	t2.setDaemon(True)
	t2.start()
	while True:
		time.sleep(1)
	t1.join()
	t2.join()

except (KeyboardInterrupt, SystemExit):
	sys.exit()

