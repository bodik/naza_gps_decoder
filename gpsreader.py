#!/usr/bin/python

import logging, json, sys, time
import nazagpsdecoder
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)

decoder = NazaGpsDecoder.NazaGpsDecoder()
while True:
	gmsg = decoder.readMessage("gps")
	gmsg = decoder.readMessage("com")
	logging.info(gmsg)
	sleep(1)
