import serial
import struct
import math
import datetime, time

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
class NazaGpsDecoder:
	messageTypes = { 0x10: "gps", 0x20: "com", 0x30: "ver" }
	fixTypes = { 0: "nolock", 2: "2d", 3: "3d" }

	def __init__(self, device="/dev/ttyAMA0", baudrate=115200, timeout=3.0):
		self.magXMin = 0
		self.magXMax = 0
		self.magYMin = 0
		self.magYMax = 0
		self.cs1 = 0
		self.cs2 = 0
		self.port = serial.Serial(device, baudrate=baudrate, timeout=timeout)

	def _seekMessageBeginning(self):
		while True:
			rcv = ord(self.port.read(1))
			if rcv == 0x55:
				rcv = ord(self.port.read(1))
				if rcv == 0xaa:
					return

	def _updateCs(self, data):
		self.cs1 += data; self.cs2 += self.cs1
		self.cs1 &= 0xff; self.cs2 &= 0xff

	def decodeShort(self, data, idx, mask):
		variableToHoldValue = bytearray([0,0])
		for i in range(0,2):
			variableToHoldValue[i] = data[idx+i] ^ mask
		return struct.unpack('h', variableToHoldValue)[0]

	def decodeLong(self, data, idx, mask):
		variableToHoldValue = bytearray([0,0,0,0])
		for i in range(0,4):
			variableToHoldValue[i] = data[idx+i] ^ mask
		return struct.unpack('l', variableToHoldValue)[0]

	def readRawMessage(self):
		payload = []
		self.cs1 = 0
		self.cs2 = 0

		self._seekMessageBeginning()
		mid = ord(self.port.read(1))
		mlen = ord(self.port.read(1))
		self._updateCs(mid)
		self._updateCs(mlen)
		rcv = self.port.read(mlen)
		for i in rcv:
			tmp=ord(i)
			payload.append(tmp)
			self._updateCs(tmp)
		mc1 = ord(self.port.read(1)); mc2 = ord(self.port.read(1))
		if ((mc1 == self.cs1) and (mc2 == self.cs2)): 
			check = "ok" 
		else:
			check = "fail"

		message = {
			"id": mid,
			"length": mlen, 
			"type": self.messageTypes.get(mid, "UNK0x%02x" % mid),
			"payload": payload, 
			"sum": [mc1,mc2], 
			"check": check, 
		}
		return message

	def decodeMessage(self, message):
		ret = {}
		if message["type"] == "ver":
			ret = self.decodeVerMessage(message)
		if message["type"] == "com":
			ret = self.decodeComMessage(message)
		if message["type"] == "gps":
			ret = self.decodeGpsMessage(message)
		return ret

	def readMessage(self, atype):
		if atype not in self.messageTypes.values():
			raise Exception("Not supported message type")
		#device is sending messages all the time, we have to wait for a requested type

		loop = True
		self.port.read(self.port.inWaiting())
		while loop:
			msg = self.readRawMessage()
			if msg["type"] == atype:
				loop = False
		msg["decoded"] = self.decodeMessage(msg)
		return msg


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
	def decodeVerMessage(self, message):
		decoded = {}
		decoded["fw"] = "v%d.%d.%d.%d" % (message["payload"][7], message["payload"][6], message["payload"][5], message["payload"][4])
		decoded["hw"] = "v%d.%d.%d.%d" % (message["payload"][11], message["payload"][10], message["payload"][9], message["payload"][8])
		return decoded

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
	def decodeComMessage(self, message):
		decoded = {}
		mask = message["payload"][4]
		mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7))
		x = self.decodeShort(message["payload"], 0, mask)
		y = self.decodeShort(message["payload"], 2, mask)
		if(x > self.magXMax):
			self.magXMax = x
		if(x < self.magXMin):
			self.magXMin = x
		if(y > self.magYMax):
			self.magYMax = y
		if(y < self.magYMin):
			self.magYMin = y
		headingNc = (-1 * math.atan2(y - ((self.magYMax + self.magYMin) / 2), x - ((self.magXMax + self.magXMin) / 2)) * 180.0) / math.pi;
		if(headingNc < 0):
			headingNc += 360.0;
		decoded["heading"] = headingNc
		return decoded

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
	def decodeGpsMessage(self, message):
		decoded = {}
		# respecting numbering in NazaDecoderLib.c, fileds numbered from 1, with headers >> -5
		mask = message["payload"][55]

		time = self.decodeLong(message["payload"], 0, mask)
		decoded["time"] = time

		second = time & 0x3f
	        time >>= 6

	        minute = time & 0x3f
	        time >>= 6

	        hour = time & 0x0f
	        time >>= 4

	        day = time & 0x1f
	        if(hour > 7):
			day += 1
	        time >>= 5

		month = time & 0x0f
	        time >>= 4

	        year = (time & 0x3f) + 2000 # ?? << 

		decoded["datetime"] = str(datetime.datetime(year=year, month=month, day=day, hour=hour, minute=minute, second=second))

		decoded["lon"] = self.decodeLong(message["payload"], 4, mask) / 10000000.0
		decoded["lat"] = self.decodeLong(message["payload"], 8, mask) / 10000000.0
		decoded["gpsAlt"] = self.decodeLong(message["payload"], 12, mask) / 1000.0
		decoded["ha"] = self.decodeLong(message["payload"], 16, mask) / 1000.0
		decoded["va"] = self.decodeLong(message["payload"], 20, mask) / 1000.0
		##decoded["unk24-25-26-27"] = [ message["payload"][24], message["payload"][25], message["payload"][26], message["payload"][27] ]
	
		decoded["nVel"] = self.decodeLong(message["payload"], 28, mask) / 1000.0
		decoded["eVel"] = self.decodeLong(message["payload"], 32, mask) / 1000.0
		decoded["dVel"] = self.decodeLong(message["payload"], 36, mask) / 1000.0	#raw

		#decoded["gpsVsi"] = -1 * decodeLong(message["payload"], 36, mask) / 100.0 #nazadecoder
		decoded["spd"] = math.sqrt( (decoded["nVel"]*decoded["nVel"]) + (decoded["eVel"]*decoded["eVel"]) )
		decoded["cog"] = math.atan2(decoded["eVel"], decoded["nVel"]) * 180.0 / math.pi
		if(decoded["cog"] < 0):
			decoded["cog"] += 360.0

		decoded["pdop"] = self.decodeShort(message["payload"], 40, mask) / 100.0
		decoded["vdop"] = self.decodeShort(message["payload"], 42, mask) / 100.0
		decoded["ndop"] = self.decodeShort(message["payload"], 44, mask) / 100.0
		decoded["edop"] = self.decodeShort(message["payload"], 46, mask) / 100.0
		decoded["hdop"] = math.sqrt( (decoded["ndop"]*decoded["ndop"]) + (decoded["edop"]*decoded["edop"]) )

		decoded["satelites"] = message["payload"][48]
		##decoded["unk49"] = message["payload"][49]
		decoded["fix_type"] = message["payload"][50]^mask
		decoded["fix"] = self.fixTypes.get(decoded["fix_type"], 'UNKNOWN')
		##decoded["unk51"] = message["payload"][51]
		decoded["fix_flags"] = message["payload"][52] ^ mask
		#if((fix != NO_FIX) && (fixFlags & 0x02)) fix = FIX_DGPS
		##decoded["unk53-54"] = [message["payload"][53], message["payload"][54]]
		decoded["mask"] = message["payload"][54]
		decoded["seq"] = struct.unpack('h', bytearray( [message["payload"][56],message["payload"][57]] ) )[0]

		return decoded


if __name__ == '__main__':
	decoder = NazaGpsDecoder()
	a = decoder.readRawMessage()
	print "SELFTEST",a
	print "SELFTEST",decoder.decodeMessage(a)
	print "SELFTEST",decoder.readMessage("ver")
	print "SELFTEST",decoder.readMessage("com")
	print "SELFTEST",decoder.readMessage("gps")

