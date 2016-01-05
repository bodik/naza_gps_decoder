import serial
import struct
import math
import datetime, time

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

	def readMessage(self):
		payload = []
		self.cs1 = 0
		self.cs2 = 0

		_seekMessageBeginning()
		mid = ord(self.port.read(1))
		mlen = ord(self.port.read(1))
		_updateCs(mid)
		_updateCs(mlen)
		rcv = port.read(mlen)
		for i in rcv:
			tmp=ord(i)
			payload.append(tmp)
			_updateCs(tmp)
		mc1 = ord(self.port.read(1)); mc2 = ord(self.port.read(1))
		if ((mc1 == cs1) and (mc2 == cs2)): 
			check = "ok" 
		else:
			check = "fail"
		message = {
			"id": mid,
			"length": mlen, 
			"type": messageTypes.get(mid, "UNK0x%02x" % mid),
			"payload": payload, 
			"sum": [mc1,mc2], 
			"check": check, 
		}
		return message

	def decodeMessage(self, message):
		decoded = {}
		if not "type" in message:
			return decoded
		if message["type"] == "ver":
			decoded = decodeVerMessage(message)
		if message["type"] == "com":
			decoded = decodeComMessage(message)
		if message["type"] == "gps":
			decoded = decodeGpsMessage(message)
		return decoded


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


	def readGpsMessage(self):
	def readVerMessage(self):
	def readComMessage(self):
		



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

	def decodeLong(self, data, idx, mask)
		variableToHoldValue = bytearray([0,0,0,0])
		for i in range(0,4):
			variableToHoldValue[i] = data[idx+i] ^ mask
		return struct.unpack('l', variableToHoldValue)[0]

