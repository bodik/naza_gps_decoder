#!/usr/bin/python
#
# naza gps serial decoder, taken from 
#	http://www.rcgroups.com/forums/showthread.php?t=1995704a
#	https://github.com/mandersonian/minimosd-naza-frsky/blob/master/NazaDecoderLib.cppa
#	https://developer.mbed.org/users/garfield38/code/NazaDecoder/rev/b0ba4e08a18c
# serial line: 115200 baud
# message struct: 55 AA ID LE <payload> ZZ ZZ
#	0x55 0xAA .. header
# 	LE .. mesage length
#	ID 0x10 which contains GPS data, sent every 250ms (refer to post #15 for more details)
#	ID 0x20 which contains compass data, sent every 30ms (refer to post #62 for more details)
#	ID 0x30 which contains GPS module version numbers, sent every 2s (refer to post#120 for more details)

import logging, json
import serial
import math, struct
import sys, datetime, time

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s', stream=sys.stdout)
msgtypes = { 0x10: "gps", 0x20: "com", 0x30: "ver" }
fixtypes = { 0: "nolock", 2: "2d", 3: "3d" }
magXMin = 0; magXMax = 0; magYMin = 0; magYMax = 0;

import threading
message_lock = threading.Lock()
message = {}
message["gps"] = {}
message["com"] = {}
message["ver"] = {}

def updatecs(din, cs1, cs2):
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
	msg["decoded"]["version"] = "v%d.%d.%d.%d" % (msg["payload"][7], msg["payload"][6],msg["payload"][5], msg["payload"][4])
	msg["decoded"]["hw"] = "v%d.%d.%d.%d" % (msg["payload"][11], msg["payload"][10],msg["payload"][9], msg["payload"][8])


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
	mask = msg["payload"][4]
	mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7))
	x = decode_short(msg["payload"], 0, mask)
	y = decode_short(msg["payload"], 2, mask)
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
	# respecting numbering in NazaDecoderLib.c, fileds numbered from 1, with headers >> -5
	mask = msg["payload"][55]

	time = decode_long(msg["payload"], 0, mask)
	msg["decoded"]["time"] = time
	msg["decoded"]["bintime"] = bin(time)
	second = time & 0x3f
        time >>= 6
        minute = time & 0x3f
        time >>= 6
        hour = time & 0x0f
        time >>= 4
        day = time & 0x1f
        time >>= 5
        if(hour > 7):
		day += 1
	month = time & 0x0f
        time >>= 4
        year = (time & 0x3f) + 2000 # ?? << 
	msg["decoded"]["datetime"] = str(datetime.datetime(year=year, month=month, day=day, hour=hour, minute=minute, second=second))

	msg["decoded"]["lon"] = decode_long(msg["payload"], 4, mask) / 10000000.0
	msg["decoded"]["lat"] = decode_long(msg["payload"], 8, mask) / 10000000.0
	msg["decoded"]["gpsAlt"] = decode_long(msg["payload"], 12, mask) / 1000.0
	msg["decoded"]["ha"] = decode_long(msg["payload"], 16, mask) / 1000.0
	msg["decoded"]["va"] = decode_long(msg["payload"], 20, mask) / 1000.0
	##msg["decoded"]["unk24-25-26-27"] = [ msg["payload"][24], msg["payload"][25],msg["payload"][26],msg["payload"][27] ]
	
	msg["decoded"]["nVel"] = decode_long(msg["payload"], 28, mask) / 1000.0
	msg["decoded"]["eVel"] = decode_long(msg["payload"], 32, mask) / 1000.0
	msg["decoded"]["dVel"] = decode_long(msg["payload"], 36, mask) / 1000.0	#raw

	#msg["decoded"]["gpsVsi"] = -1 * decode_long(msg["payload"], 36, mask) / 100.0 #nazadecoder
	msg["decoded"]["spd"] = math.sqrt( (msg["decoded"]["nVel"]*msg["decoded"]["nVel"]) + (msg["decoded"]["eVel"]*msg["decoded"]["eVel"]) )
	msg["decoded"]["cog"] = math.atan2(msg["decoded"]["eVel"], msg["decoded"]["nVel"]) * 180.0 / math.pi
	if(msg["decoded"]["cog"] < 0):
		msg["decoded"]["cog"] += 360.0

	msg["decoded"]["pdop"] = decode_short(msg["payload"], 40, mask) / 100.0
	msg["decoded"]["vdop"] = decode_short(msg["payload"], 42, mask) / 100.0
	msg["decoded"]["ndop"] = decode_short(msg["payload"], 44, mask) / 100.0
	msg["decoded"]["edop"] = decode_short(msg["payload"], 46, mask) / 100.0
	msg["decoded"]["hdop"] = math.sqrt( (msg["decoded"]["ndop"]*msg["decoded"]["ndop"]) + (msg["decoded"]["edop"]*msg["decoded"]["edop"]) )

	msg["decoded"]["satelites"] = msg["payload"][48]
	##msg["decoded"]["unk49"] = msg["payload"][49]
	msg["decoded"]["fix_type"] =  fixtypes.get((msg["payload"][50]^mask), 'UNKNOWN' )
	##msg["decoded"]["unk51"] = msg["payload"][51]
	msg["decoded"]["fix_flags"] = msg["payload"][52] ^ mask
	#if((fix != NO_FIX) && (fixFlags & 0x02)) fix = FIX_DGPS
	##msg["decoded"]["unk53-54"] = [msg["payload"][53], msg["payload"][54]]
	msg["decoded"]["mask"] = msg["payload"][54]
	msg["decoded"]["seq"] = struct.unpack('h', bytearray([msg["payload"][56],msg["payload"][57]]) )[0]


	return



# 55 AA ID LE DATA CS CS
def recv_message(port):
	global message
	cs1 = 0; cs2 = 0;
	payload = []

	mid = ord(port.read(1)); mlen = ord(port.read(1))
	updatecs(mid, cs1, cs2); updatecs(mlen, cs1, cs2)
	rcv = port.read(mlen)
	for i in rcv:
		t=ord(i)
		payload.append(t)
		updatecs(t, cs1, cs2)
	mc1 = ord(port.read(1)); mc2 = ord(port.read(1))
	if ((mc1 == cs1) and (mc2 == cs2)): 
		check = "ok" 
	else:
		check = "fail"

	msg = { "length": mlen, "id": mid, "type": msgtypes.get(mid, "UNK0x%02x" % mid), "payload": payload, "sum": [mc1,mc2], "check": check, "decoded":dict() }
	if msg["type"] == "ver":
		decode_ver(msg)
	if msg["type"] == "com":
		decode_com(msg)
		#logging.debug(msg["decoded"])
	if msg["type"] == "gps":
		decode_gps(msg)
		#logging.debug(msg["decoded"])

	#logging.info(msg)
	message_lock.acquire()
	message[msg["type"]] = msg
	message_lock.release()
	return msg


def data_reader():
	try:
		port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)
		while True:
			rcv = ord(port.read(1))
			if rcv == 0x55:
				rcv = ord(port.read(1))
				if rcv == 0xaa:
					recv_message(port)
			else:
				#logging.warn("rcvd 0x%02x" % rcv)
				pass
	except Exception as e:
		#logging.warn("stopping data_reader")
		pass
		

def data_printer():
	global message, message_lock
	try:
		while True:
			message_lock.acquire()
			if message["gps"]!={} and message["com"]!={} and message["ver"]!={}:
				logging.info("%s %s %s %s %s %s %s" % (
					message["gps"]["decoded"]["datetime"], 
					message["gps"]["decoded"]["bintime"], 
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

