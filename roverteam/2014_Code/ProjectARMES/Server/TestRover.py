import sys
import thread
import time
import threading
import socket
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *
import serial

# SEND TELEMETRY BACK TO SERVERCOMM
SERVERCOMMS_RECV_UDP_IP = "192.168.1.103"
SERVERCOMMS_RECV_UDP_PORT = 5022
# LISTEN FOR SERVERCOMM MSGS
ROVERCOMMS_RECV_UDP_IP = "192.168.1.103"
ROVERCOMMS_RECV_UDP_PORT = 5021

ROVER_EMERGENCY_UDP_IP = "192.168.1.20"
ROVER_EMERGENCY_UDP_PORT=10000
ROVER_EMERGENCY_RECV_UDP_IP = "192.168.1.103"
ROVER_EMERGENCY_RECV_UDP_PORT=10000

class RoverComms:
	#----------------		1. INIT COMMANDS		----------------#
	def __init__(self):# -110.813125109,38.4374592146
		self.runThreads = True
		self.rover = RoverModel()
		self.rover.max_timeout = 300
		# self.rover.ser = serial.Serial('/dev/ttyUSB0',115200,xonxoff=True,rtscts=True,timeout=0.1)
		self.rover.set(NS_GPS_LATI, -110.813125109)
		self.rover.set(NS_GPS_LONG, 38.4374592146)
		self._rover_Comms = UniCastComms(ROVERCOMMS_RECV_UDP_IP, ROVERCOMMS_RECV_UDP_PORT)
		self._emrover_Comms = UniCastComms(ROVER_EMERGENCY_RECV_UDP_IP, ROVER_EMERGENCY_RECV_UDP_PORT)

		# print "UDP target IP:", UDP_IP
		# print "UDP target port:", UDP_PORT
		self.recvThread = RecvThread(self, 1, self._uni_RecvFunction, self._uni_ProcessCommand, 0.01)
		self.sendThread = self.SerialThread(self, 2, self._uni_telemetrySerialFunct, self._uni_closeSerial, 0.01)
	
	def startThreads(self):
		self.recvThread.start()
		self.sendThread.start()

	def endThreads(self):
		print 'end threads'
		self.runThreads = False
		# self._multi_SendFunction('end')
		self.recvThread.isRunning = False
		self._uni_SendFunction('end')
		self.sendThread.isRunning = False
		self._rover_Comms.sendFunction('end', ROVERCOMMS_RECV_UDP_IP, ROVERCOMMS_RECV_UDP_PORT)
		self._rover_Comms.sendFunction('end', ROVER_EMERGENCY_RECV_UDP_IP, ROVER_EMERGENCY_RECV_UDP_PORT)

	def closeEvent(self, event):
		#self.screenVideo.streamer.quit()
		self.endThreads()
		self._uni_closeSerial()
		event.accept()
		sys.exit()

	#----------------		2. SERVER MULTICAST FUNCTIONS		----------------#

	#----------------		3. SERVER UNICAST FUNCTIONS		----------------#
	def _uni_SendFunction(self, msg):
		self._rover_Comms.sendFunction(msg, SERVERCOMMS_RECV_UDP_IP, SERVERCOMMS_RECV_UDP_PORT)

	def _uni_RecvFunction(self):
		data, addr = self._rover_Comms.recvFunction()
		return data, addr

	def _uni_ProcessCommand(self, num, msg):
		if msg[0] == 'end':
			return
		#1. decode msg and check if there is a difference
		# isDiff = self.rover.updateTelemetry(msg[0])# msg = (data,addr)
		#2. encode for serial
		path,value,chksum = self.rover.decode(msg[0])
		print 'path',path,NS_NAVCOM, path[0]==NS_NAVCOM
		if path[0] == NS_NAVCOM:
			print 'navcom:',msg[0],'send!'
			# self.rover.ser.write(msg[0])
			self._emrover_Comms.sendFunction(msg[0], ROVER_EMERGENCY_UDP_IP,ROVER_EMERGENCY_UDP_PORT)
			# LOOPBACK TEST
			# self._uni_SendFunction(msg[0])
		# elif path[0] == NS_DRIVE:
		# 	print 'serial:',msg[0],'send!'
		# 	self._emrover_Comms.sendFunction(msg[0], ROVER_EMERGENCY_UDP_IP,ROVER_EMERGENCY_UDP_PORT)
		elif path[0] == NS_SCIENCE:
			print 'serial:',msg[0],'send!'
			self._emrover_Comms.sendFunction(msg[0], ROVER_EMERGENCY_UDP_IP,ROVER_EMERGENCY_UDP_PORT)
		elif path[0] == NS_ARM:
			print 'serial:',msg[0],'send!'
			self._emrover_Comms.sendFunction(msg[0], ROVER_EMERGENCY_UDP_IP,ROVER_EMERGENCY_UDP_PORT)
		elif self.rover.validatePkt(msg[0]):
			if self.rover.isDifferent(msg[0]):
				print 'serial:',msg[0],'send!'
				# self.rover.ser.write(msg[0])
				self._emrover_Comms.sendFunction(msg[0], ROVER_EMERGENCY_UDP_IP,ROVER_EMERGENCY_UDP_PORT)
			# LOOPBACK TEST
			# if self.rover.updateTelemetry(msg[0]):
			# 	self._uni_SendFunction(msg[0])
		else:
			print 'serial:',msg[0]
		#3. send serial
		# self._uni_SendFunction(msg[0])
		pass

	def _uni_TestFunction(self, num, msg):
		print 'test',num,':',msg

	def _uni_telemetrySerialFunct(self, num):
		# msg = self.rover.receivePacket()
		data, addr = self._emrover_Comms.recvFunction() # buffer size is 1024 bytes
		msg = data
		print 'recvTelemetry',msg,msg == self.rover.heartbeat
		if msg == self.rover.heartbeat:
			return
		if msg == 'end':
			return
		path,value,chksum = self.rover.decode(msg)
		if path[0] == NS_NAVCOM:
			print ''
			self._uni_SendFunction(msg)
		elif self.rover.validatePkt(msg):
			# print 'validated'
			if self.rover.updateTelemetry(msg):
				# print 'updated'
				self._uni_SendFunction(msg)
		pass

	def _uni_closeSerial(self):
		print 'end serial'
		# self.rover.serial.close()

	#----------------		AUXILLARY FUNCTIONS		----------------#
	def _uni_telemetryFunct(self, num):
		pass

	class SerialThread(threading.Thread):
		def __init__(self, parent, num, threadFunc, closeFunc, delay):
			threading.Thread.__init__(self)
			self.parent = parent
			self.num = num
			self.threadFunc = threadFunc
			self.closeFunc = closeFunc
			self.delay = delay

		def run(self):
			print 'thread',self.num,':','started'
			isRunning = True
			while self.parent.runThreads:
				self.threadFunc(self.num)
				time.sleep(self.delay)
				# break

			print 'thread',self.num,':','ended'

def main():
	print "Rover Test main"
	server = RoverComms()
	server.startThreads()
	while True:
		try:
			time.sleep(1)
		except KeyboardInterrupt:
			print "Ctrl+C pressed, exitting"
			server.endThreads()
			sys.exit()
			pass

if __name__ == "__main__":
	main()

# Fasteners: Leland Industries : 416-291-5308 Mike extension 222
# cathyriani@hotmail.com - First Robotics Mentorship