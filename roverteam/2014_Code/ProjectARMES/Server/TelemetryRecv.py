# gps,4979.0,38.37369,110.70870,1309.3
# gps,[second since midnight[UTC]],[latitude],[longitude],[altitude]\n'
import thread
import time
import threading
import socket
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *

# RECV FROM ROVER
ROVER_TELE_RECV_UDP_IP = "192.168.1.103"
ROVER_TELE_RECV_UDP_PORT = 5055
# SEND TO ROVER 
ROVER_TELE_SEND_UDP_IP = "192.168.1.103"
ROVER_TELE_SEND_UDP_PORT = 5056

# SERVER_RECV_UDP_IP = "192.168.1.103"
# SERVER_RECV_UDP_PORT = 10000

# TELEMETRYRECV RECV TELEMETRY FROM GUIs
TELEMETRY_RECV_UDP_IP = "192.168.1.103"
TELEMETRY_RECV_UDP_PORT = 30000
# OPERATOR GUI SEND
TELEMETRY1_SEND_UDP_IP = "192.168.1.103"
TELEMETRY1_SEND_UDP_PORT = 30001
# NAVIGATOR GUI SEND
TELEMETRY2_SEND_UDP_IP = "192.168.1.103"
TELEMETRY2_SEND_UDP_PORT = 30003
# OPERATOR GUI SEND
TELEMETRY3_SEND_UDP_IP = "192.168.1.104"
TELEMETRY3_SEND_UDP_PORT = 30001
# NAVIGATOR GUI SEND
TELEMETRY4_SEND_UDP_IP = "192.168.1.104"
TELEMETRY4_SEND_UDP_PORT = 30003

# TELEMETRY_RECV_UDP_IP = "127.0.0.1"
# TELEMETRY_RECV_UDP_PORT = 30000
# TELEMETRY1_SEND_UDP_IP = "127.0.0.1"
# TELEMETRY1_SEND_UDP_PORT = 30001
# TELEMETRY2_SEND_UDP_IP = "127.0.0.1"
# TELEMETRY2_SEND_UDP_PORT = 30003

THREAD_DELAY = 0.01

class RoverComms:
	#----------------		1. INIT COMMANDS		----------------#
	def __init__(self):
		self.rover = RoverModel()
		self._server_clients = UniCastComms(TELEMETRY_RECV_UDP_IP, TELEMETRY_RECV_UDP_PORT)
		self._rover_telemetry = UniCastComms(ROVER_TELE_RECV_UDP_IP, ROVER_TELE_RECV_UDP_PORT)
		self.runThreads = True
		self.server_MultiCastThread = RecvThread(self, 1, self._Server_MultiCast_RecvFunction, self._Server_MultiCast_TestFunction, THREAD_DELAY)
		self.rover_UniCastThread = RecvThread(self, 3, self._Rover_UniCast_RecvFunction, self._Rover_UniCast_ProcessTelemetry, THREAD_DELAY)
		self.clients = [
			[TELEMETRY1_SEND_UDP_IP,TELEMETRY1_SEND_UDP_PORT],
			[TELEMETRY2_SEND_UDP_IP,TELEMETRY2_SEND_UDP_PORT],
			[TELEMETRY3_SEND_UDP_IP,TELEMETRY3_SEND_UDP_PORT],
			[TELEMETRY4_SEND_UDP_IP,TELEMETRY4_SEND_UDP_PORT]
		]

	def startThreads(self):
		self.server_MultiCastThread.start()
		self.rover_UniCastThread.start()

	def endThreads(self):
		print 'end Threads!'
		self.runThreads = False
		self._rover_telemetry.sendFunction('end', TELEMETRY_RECV_UDP_IP, TELEMETRY_RECV_UDP_PORT)
		self._rover_telemetry.sendFunction('end', ROVER_TELE_RECV_UDP_IP, ROVER_TELE_RECV_UDP_PORT)

	#----------------		2. SERVER MULTICAST FUNCTIONS		----------------#
	def _Server_MultiCast_RecvFunction(self):
		return self._server_clients.recvFunction()

	def _Server_MultiCast_SendFunction(self, msg):
		for i in range(len(self.clients)):
			# print 'sendto',self.clients[i],msg
			self._server_clients.sendFunction(msg, self.clients[i][0],self.clients[i][1])
		# self._server_clients.sendFunction(msg, TELEMETRY1_SEND_UDP_IP, TELEMETRY1_SEND_UDP_PORT)
		# self._server_clients.sendFunction(msg, TELEMETRY2_SEND_UDP_IP, TELEMETRY2_SEND_UDP_PORT)

	def _Server_MultiCast_TestFunction(self, num, msg):
		pass
		# print 'thread(multi)',num,' got:',msg

	#----------------		ROVER UNICAST TESTING FUNCTIONS		----------------#
	# NEVER USE
	def _Rover_UniCast_SendFunction(self, msg):
		self._rover_telemetry.sendFunction(msg, (ROVER_TELE_SEND_UDP_IP, ROVER_TELE_SEND_UDP_PORT))

	def _Rover_UniCast_RecvFunction(self):
		data,addr = self._rover_telemetry.recvFunction()
		return data

	def _Rover_UniCast_ProcessTelemetry(self, num, msg):
		# print 'thread( uni )',num,' got:',msg
		arr = msg.split(',')
		# gps,[second since midnight[UTC]],[latitude],[longitude],[altitude]\n'
		if len(arr) == 2:# "compass",heading[0-359]
			heading = int(arr[1])
			sender = self.rover.encode(NS_COMPASS,arr[1])
			self._Server_MultiCast_SendFunction(sender)
			print 'send',sender
		elif len(arr) == 5 and arr[0] != 'acceleration':
			
			code = arr[0]
			time = float(arr[1])
			lat  = float(arr[2])
			lon  = float(arr[3])
			alt  = float(arr[4])
			print time,lat,lon,alt
			# sender = 'C'+'01'+'_'++'E'
			sender = self.rover.encode(NS_GPS,arr[2]+','+arr[3]+','+arr[4])
			self._Server_MultiCast_SendFunction(sender)
			print 'send',sender

		# if self.rover.updateTelemetry(msg):
		# 	self._Server_MultiCast_SendFunction(msg)
		# 	print 'update!',msg

	def _Rover_UniCast_TestFunction(self, num, msg):
		print 'thread( uni )',num,' got:',msg

	#----------------		AUXILLARY FUNCTIONS		----------------#

def main():
	print "RoverComms main"
	server = RoverComms()
	server.startThreads()
	while True:
		try:
			time.sleep(1)
		except KeyboardInterrupt:
			print "Ctrl+C pressed, exitting"
			server.endThreads()
			break
			pass


if __name__ == "__main__":
	main()