#Communications to the rover. This is the only code that actually talks to the rover. 
#All communications goes through this.

# Server Multicast = pushing Telemetry to all clients (one way only)
# Server UDP unicast = commands and special operations (two way)

import sys
import thread
import time
import threading
import socket
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *

# RECV TELEMETRY FROM ROVERCOMMS
SERVERCOMMS_RECV_UDP_IP = "192.168.1.103"
SERVERCOMMS_RECV_UDP_PORT = 5022
# SEND CMDS TO ROVERCOMMS
ROVERCOMMS_RECV_UDP_IP = "192.168.1.103"
ROVERCOMMS_RECV_UDP_PORT = 5021
# RECV CMDS FROM CONTROLLER
SERVER_RECV_UDP_IP = "192.168.1.103"
SERVER_RECV_UDP_PORT = 20000

# ROVER_RECV_UDP_IP = "192.168.1.103"
# ROVER_RECV_UDP_PORT = 5005
# ROVER_SEND_UDP_IP = "192.168.1.103"
# ROVER_SEND_UDP_PORT = 5006

# RECV TELEMETRY FROM GUIs
TELEMETRY_RECV_UDP_IP = "192.168.1.103"
TELEMETRY_RECV_UDP_PORT = 30002
# SEND TELEMETRY TO GUIs
TELEMETRY1_SEND_UDP_IP = "192.168.1.103"
TELEMETRY1_SEND_UDP_PORT = 30001
# NAVIGATION_GUI
TELEMETRY2_SEND_UDP_IP = "192.168.1.103"
TELEMETRY2_SEND_UDP_PORT = 30003
# SEND TELEMETRY TO GUIs
TELEMETRY3_SEND_UDP_IP = "192.168.1.104"
TELEMETRY3_SEND_UDP_PORT = 30001
# NAVIGATION_GUI
TELEMETRY4_SEND_UDP_IP = "192.168.1.104"
TELEMETRY4_SEND_UDP_PORT = 30003

THREAD_DELAY = 0.01

class RoverComms:
	#----------------		1. INIT COMMANDS		----------------#
	def __init__(self):
		self.rover = RoverModel()
		self._server_TelemetryComms = UniCastComms(TELEMETRY_RECV_UDP_IP, TELEMETRY_RECV_UDP_PORT)
		self._server_CommandComms = UniCastComms(SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
		self._rover_Comms = UniCastComms(SERVERCOMMS_RECV_UDP_IP, SERVERCOMMS_RECV_UDP_PORT)
		self.runThreads = True
		self.server_MultiCastThread = RecvThread(self, 1, self._Server_MultiCast_RecvFunction, self._Server_MultiCast_TestFunction, THREAD_DELAY)
		self.server_UniCastThread = RecvThread(self, 2, self._Server_UniCast_RecvFunction, self._Server_UniCast_CommandFunction, THREAD_DELAY)
		self.rover_UniCastThread = RecvThread(self, 3, self._Rover_UniCast_RecvFunction, self._Rover_UniCast_PushFunction, THREAD_DELAY)
		self.clients = [
			[TELEMETRY1_SEND_UDP_IP,TELEMETRY1_SEND_UDP_PORT],
			[TELEMETRY2_SEND_UDP_IP,TELEMETRY2_SEND_UDP_PORT],
			[TELEMETRY3_SEND_UDP_IP,TELEMETRY3_SEND_UDP_PORT],
			[TELEMETRY4_SEND_UDP_IP,TELEMETRY4_SEND_UDP_PORT]
		]

	def startThreads(self):
		self.server_MultiCastThread.start()
		self.server_UniCastThread.start()
		self.rover_UniCastThread.start()

	def endThreads(self):
		print 'end Threads!'
		self.runThreads = False
		self._server_TelemetryComms.sendFunction('end', TELEMETRY_RECV_UDP_IP, TELEMETRY_RECV_UDP_PORT)
		self._Server_UniCast_SendFunction('end', SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
		self._Server_UniCast_SendFunction('end', SERVERCOMMS_RECV_UDP_IP, SERVERCOMMS_RECV_UDP_PORT)# Not exactly the best, but Rover_SendFunction is always forward facing.

	def closeEvent(self, event):
		#self.screenVideo.streamer.quit()
		self.endThreads()
		self._server_TelemetryComms.close()
		self._server_CommandComms.close()
		self._rover_Comms.close()
		event.accept()
		sys.exit()

	#----------------		2. SERVER MULTICAST FUNCTIONS		----------------#
	def _Server_MultiCast_RecvFunction(self):
		return self._server_TelemetryComms.recvFunction()

	def _Server_MultiCast_SendFunction(self, msg):
		for i in range(len(self.clients)):
			self._server_TelemetryComms.sendFunction(msg, self.clients[i][0],self.clients[i][1])
		# self._server_TelemetryComms.sendFunction(msg, TELEMETRY1_SEND_UDP_IP, TELEMETRY1_SEND_UDP_PORT)
		# self._server_TelemetryComms.sendFunction(msg, GUI_SEND_UDP_IP, GUI_SEND_UDP_PORT)

	def _Server_MultiCast_TestFunction(self, num, msg):
		pass
		# print 'thread(multi)',num,' got:',msg

	#----------------		3. SERVER UNICAST FUNCTIONS		----------------#
	def _Server_UniCast_SendFunction(self, msg, ip, port):
		self._server_CommandComms.sendFunction(msg, ip, port)

	def _Server_UniCast_RecvFunction(self):
		data,addr = self._server_CommandComms.recvFunction()
		# print 'data',data
		return data

	def _Server_UniCast_CommandFunction(self, num, msg):
		self._Rover_UniCast_SendFunction(msg)

	def _Server_UniCast_TestFunction(self, num, msg):
		print 'thread( uni )',num,' got:',msg


	#----------------		ROVER UNICAST TESTING FUNCTIONS		----------------#
	def _Rover_UniCast_SendFunction(self, msg):
		self._rover_Comms.sendFunction(msg, ROVERCOMMS_RECV_UDP_IP, ROVERCOMMS_RECV_UDP_PORT)

	def _Rover_UniCast_RecvFunction(self):
		data,addr = self._rover_Comms.recvFunction()
		return data

	def _Rover_UniCast_PushFunction(self, num, msg):
		#print 'thread( uni )',num,' got:',msg
		# if self.rover.updateTelemetry(msg):
		self._Server_MultiCast_SendFunction(msg)
		print 'update!',msg

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
			# server._server_TelemetryComms.close()
			# server._server_CommandComms.close()
			# server._rover_Comms.close()
			sys.exit()
			pass

if __name__ == "__main__":
	main()