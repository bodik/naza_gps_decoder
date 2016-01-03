#!/usr/bin/python
#
# naza gps serial decoder, taken from http://www.rcgroups.com/forums/showthread.php?t=1995704
# serial line: 115200 baud
# message struct: 55 AA ID LE <payload> ZZ ZZ
#	0x55 0xAA .. header
# 	LE .. mesage length
#	ID 0x10 which contains GPS data, sent every 250ms (refer to post #15 for more details)
#	ID 0x20 which contains compass data, sent every 30ms (refer to post #62 for more details)
#	ID 0x30 which contains GPS module version numbers, sent every 2s (refer to post#120 for more details)

import logging
import json
import serial
import math
import struct
import sys

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)
msgtypes = { 0x10: "gps", 0x20: "com", 0x30: "ver" }
cs1 = 0; cs2 = 0; magXMin = 0; magXMax = 0; magYMin = 0; magYMax = 0;

def updatecs(din):
	global cs1, cs2
	cs1 += din; cs2 += cs1; cs1 &= 0xff; cs2 &= 0xff
	#logging.debug("0x%02x 0x%02x" % (cs1, cs2))


def decode_short(data, idx, mask):
	variableToHoldValue = bytearray([0,0])
	for i in range(0,2):
		variableToHoldValue[i] = data[idx+i] ^ mask
	return struct.unpack('h', variableToHoldValue)[0]


def decode_long(data, idx, mask):
	variableToHoldValue = bytearray([0,0,0,0])
	for i in range(0,4):
		variableToHoldValue[i] = data[idx+i] ^ mask
	return struct.unpack('l', variableToHoldValue)[0]


#http://www.rcgroups.com/forums/showpost.php?p=27058649&postcount=120a
#Message 0x30
#	This is still to be confirmed but I believe the 0x30 message carries the GPS module hardware id and firmware version.
#	55 AA 30 0C XX XX XX XX FW FW FW FW HW HW HW HW CS CS 
#	Note that you need to read version numbers backwards (02 01 00 06 means v6.0.1.2)
#	HEADER
#		BYTE 1-2: message header - always 55 AA
#		BYTE 3: message id (0x30 for GPS module versions message)
#		BYTE 4: length of the payload (0x0C or 12 decimal for 0x30 message)
#	PAYLOAD
#		BYTE 5-8" ??? (seems to be always 0)
#		BYTE 9-12 (FW): firmware version
#		BYTE 13-16 (HW): hardware id
#	CHECKSUM
#		BYTE 17-18 (CS): checksum, calculated the same way as for uBlox binary messages
def decode_ver(msg):
	msg["decoded"]["version"] = "v%d.%d.%d.%d" % (msg["mdata"][7], msg["mdata"][6],msg["mdata"][5], msg["mdata"][4])
	msg["decoded"]["hw"] = "v%d.%d.%d.%d" % (msg["mdata"][11], msg["mdata"][10],msg["mdata"][9], msg["mdata"][8])


#http://www.rcgroups.com/forums/showpost.php?p=26248426&postcount=62
# Message 0x20 
#	message contains compass data. Here is the structure of the message, fields marked with XX I'm not sure about yet. The others will be described below.
#	55 AA 20 06 CX CX CY CY CZ CZ CS CS
#	Values in the message are stored in little endian.
#	HEADER
#		BYTE 1-2: message header - always 55 AA
#		BYTE 3: message id (0x20 for compass message)
#		BYTE 4: length of the payload (0x06 or 6 decimal for 0x20 message)
#	PAYLOAD
#		BYTE 5-6 (CX): compass X axis data (signed) - see comments below
#		BYTE 7-8 (CY): compass Y axis data (signed) - see comments below
#		BYTE 9-10 (CZ): compass Z axis data (signed) - see comments below
#	CHECKSUM
#		BYTE 11-12 (CS): checksum, calculated the same way as for uBlox binary messages
#
#	All the bytes of the payload except 9th are XORed with a mask. Mask is calculated based on the value of the 9th byte.
#	If we index bits from LSB to MSB as 0-7 we have:
#	mask[0] = 9thByte[0] xor 9thByte[4]
#	mask[1] = 9thByte[1] xor 9thByte[5]
#	mask[2] = 9thByte[2] xor 9thByte[6]
#	mask[3] = 9thByte[3] xor 9thByte[7] xor 9thByte[0];
#	mask[4] = 9thByte[1];
#	mask[5] = 9thByte[2];
#	mask[6] = 9thByte[3];
#	mask[7] = 9thByte[4] xor 9thByte[0];
#	To calculate the heading (not tilt compensated) you need to do atan2 on the resulting y any a values, convert radians to degrees and add 360 if the result is negative.
def decode_com(msg):
	global magXMin, magXMax, magYMin, magYMax
	mask = msg["mdata"][0] #4rth in original, dunno why 9th in description
	mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7))
	x = decode_short(msg["mdata"], 0, mask)
	y = decode_short(msg["mdata"], 2, mask)
	if(x > magXMax):
		magXMax = x
	if(x < magXMin):
		magXMin = x
	if(y > magYMax):
		magYMax = y
	if(y < magYMin):
		magYMin = y
	headingNc = (-1 * math.atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0) / math.pi;
	if(headingNc < 0):
		headingNc += 360.0;
	msg["decoded"]["heading"] = headingNc


# Message 0x10
#	The 0x10 message contains GPS data. Here is the structure of the message, fields marked with XX I'm not sure about yet. The others will be described below.
#	55 AA 10 3A DT DT DT DT LO LO LO LO LA LA LA LA AL AL AL AL HA HA HA HA VA VA VA VA XX XX XX XX NV NV NV NV EV EV EV EV DV DV DV DV PD PD VD VD ND ND ED ED NS XX FT XX SF XX XX XM SN SN CS CS
#	The payload is XORed with a mask that changes over time (see below for more details). Values in the message are stored in little endian.
#	HEADER
#		BYTE 1-2: message header - always 55 AA
#		BYTE 3: message id (0x10 for GPS message)
#		BYTE 4: lenght of the payload (0x3A or 58 decimal for 0x10 message)
#	PAYLOAD
#		BYTE 5-8 (DT): date and time, see details below
#		BYTE 9-12 (LO): longitude (x10^7, degree decimal)
#		BYTE 13-16 (LA): latitude (x10^7, degree decimal)
#		BYTE 17-20 (AL): altitude (in milimeters)
#		BYTE 21-24 (HA): horizontal accuracy estimate (see uBlox NAV-POSLLH message for details)
#		BYTE 25-28 (VA): vertical accuracy estimate (see uBlox NAV-POSLLH message for details)
#		BYTE 29-32: ??? (seems to be always 0)
#		BYTE 33-36 (NV): NED north velocity (see uBlox NAV-VELNED message for details)
#		BYTE 37-40 (EV): NED east velocity (see uBlox NAV-VELNED message for details)
#		BYTE 41-44 (DV): NED down velocity (see uBlox NAV-VELNED message for details)
#		BYTE 45-46 (PD): position DOP (see uBlox NAV-DOP message for details)
#		BYTE 47-48 (VD): vertical DOP (see uBlox NAV-DOP message for details)
#		BYTE 49-50 (ND): northing DOP (see uBlox NAV-DOP message for details)
#		BYTE 51-52 (ED): easting DOP (see uBlox NAV-DOP message for details)
#		BYTE 53 (NS): number of satellites (not XORed)
#		BYTE 54: ??? (not XORed, seems to be always 0)
#		BYTE 55 (FT): fix type (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
#		BYTE 56: ??? (seems to be always 0)
#		BYTE 57 (SF): fix status flags (see uBlox NAV-SOL message for details)
#		BYTE 58-59: ??? (seems to be always 0)
#		BYTE 60 (XM): not sure yet, but I use it as the XOR mask
#		BYTE 61-62 (SN): sequence number (not XORed), once there is a lock - increases with every message. When the lock is lost later LSB and MSB are swapped with every message.
#	CHECKSUM
#		BYTE 63-64 (CS): checksum, calculated the same way as for uBlox binary messages
#	XOR mask
#		All bytes of the payload except 53rd (NS), 54th, 61st (SN LSB) and 62nd (SN MSB) are XORed with a mask. Mask is calculated based on the value of byte 53rd (NS) and 61st (SN LSB).
#		If we index bits from LSB to MSB as 0-7 we have:
#		mask[0] = 53rdByte[0] xor 61stByte[4]
#		mask[1] = 53rdByte[1] xor 61stByte[5]
#		mask[2] = 53rdByte[2] xor 61stByte[6]
#		mask[3] = 53rdByte[3] xor 61stByte[7] xor 53rdByte[0];
#		mask[4] = 53rdByte[1];
#		mask[5] = 53rdByte[2];
#		mask[6] = 53rdByte[3];
#		mask[7] = 53rdByte[0] xor 61stByte[4];
#		To simplify calculations any of the unknown bytes that when XORer seem to be always 0 (29-32, 56, 58-60) can be used as XOR mask (based on the fact that 0 XOR mask == mask). In the library I use byte 60.
#	Date and time format
#		Date (Year, Month, Day) and time (Hour, Minute, Second) are stored as little endian 32bit unsigned integer, the meaning of particular bits is as follows:
#		YYYYYYYMMMMDDDDDHHHHMMMMMMSSSSSS
#		NOTE 1: to get the day value correct you must add 1 when hour is > 7
#		NOTE 2: for the time between 16:00 and 23:59 the hour will be returned as 0-7 and there seems to be no way to differentiate between 00:00 - 07:59 and 16:00 - 23:59.
#		You wil find the uBlox binary messages specification in this document. It also contains checksum calculation algorithm.

def decode_gps(msg):
	#mdata is stripped from header >> -4
	msg["decoded"]["numer_of_satelites"] = msg["mdata"][53 - 4]
	return



# 55 AA ID LE DATA CS CS
def recv_message(port):
	global cs1, cs2
	mdata = []

	mid = ord(port.read(1)); mlen = ord(port.read(1))
	updatecs(mid); updatecs(mlen)
	rcv = port.read(mlen)
	for i in rcv:
		t=ord(i)
		mdata.append(t)
		updatecs(t)
	mc1 = ord(port.read(1)); mc2 = ord(port.read(1))
	if ((mc1 == cs1) and (mc2 == cs2)): 
		check = "ok" 
	else:
		check = "fail"

	msg = { "mlen": mlen, "mid": mid, "mtype": msgtypes.get(mid, "UNK0x%02x" % mid), "mdata": mdata, "mcsum": [mc1,mc2], "mccheck": check, "decoded":dict() }
	if msg["mtype"] == "ver":
		decode_ver(msg)
	if msg["mtype"] == "com":
		decode_com(msg)
		#logging.debug(msg["decoded"])
	if msg["mtype"] == "gps":
		decode_gps(msg)
		logging.debug(msg["decoded"])

		
	#logging.info("message %s %d %s %s" % msg["mtype"], mlen, check, repr(mdata)))
	#logging.info(msg)


#main
port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)
while True:
	rcv = ord(port.read(1))
	if rcv == 0x55:
		rcv = ord(port.read(1))
		if rcv == 0xaa:
			cs1 = 0; cs2 = 0
			recv_message(port)
	else:
		logging.warn("rcvd 0x%02x" % rcv)

