#!/usr/bin/python

import logging, json, sys, time
import nazagpsdecoder
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)

decoder = nazagpsdecoder.NazaGpsDecoder()

fmt = "%6s %20s %32s %4s %-10s %-10s %-13s %-16s"
columns = ("seq", "datetime", "bintime", "sats", "lat", "lon", "heading", "spd")
logging.info(fmt % columns)

while True:
	gmsg = decoder.readMessage("gps")
	cmsg = decoder.readMessage("com")
	logging.info(fmt % (
			gmsg["decoded"]["seq"], 
			gmsg["decoded"]["datetime"], 
			bin(gmsg["decoded"]["time"]),
			gmsg["decoded"]["satelites"], 
			gmsg["decoded"]["lat"], 
			gmsg["decoded"]["lon"], 
			cmsg["decoded"]["heading"], 
			gmsg["decoded"]["spd"], 
	) )
	time.sleep(1)

