#Communications to the rover. This is the only code that actually talks to the rover. 
#All communications goes through this.

# Server Multicast = pushing Telemetry to all clients (one way only)
# Server UDP unicast = commands and special operations (two way)

import thread
import time
import threading
import socket
import struct

class RecvThread (threading.Thread):
	def __init__(self, parent, num, recvFunc, threadFunc, delay):
		threading.Thread.__init__(self)
		self.parent = parent
		self.num = num
		self.recvFunc = recvFunc
		self.threadFunc = threadFunc
		self.delay = delay

	def run(self):
		print 'thread',self.num,':','started'
		isRunning = True
		while isRunning and self.parent.runThreads:
			try:
				msg = self.recvFunc()
				if msg == 'end':
					isRunning = False
					self.parent.endThreads()# KILL(JOIN) ALL THREADS
				else:
					self.threadFunc(self.num, msg)
				print 'recv',self.num,':',msg
				time.sleep(self.delay)
				
			except KeyboardInterrupt:
				print "Ctrl+C pressed, exitting"
				isRunning = False
				self.parent.runThreads = False

		print 'thread',self.num,':','ended'

class MultiCastComms():
	def __init__(self, ip, port, recvPort):
		self.ip = ip #MCAST_GRP
		self.port = port #MCAST_PORT
		self.recvPort = recvPort #MCAST_RECV_PORT 
		#RECV MULTICAST SOCKET
		self.recvSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.recvSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.recvSocket.bind((ip, port))
		mreq = struct.pack("4sl", socket.inet_aton(ip), socket.INADDR_ANY)
		self.recvSocket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
		#SEND MULTICAST SOCKET
		self.sendSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.sendSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

	def sendFunction(self, msg):
		self.sendSocket.sendto(msg, (self.ip, self.port))

	def recvFunction(self):
		return self.recvSocket.recv(self.recvPort)

class UniCastComms():
	def __init__(self, recvIP, recvPort):
		self.recvIP = recvIP #SERVER_RECV_UDP_IP
		self.recvPort = recvPort #SERVER_RECV_UDP_PORT
		self.recvSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)# UDP
		self.recvSocket.bind((self.recvIP, self.recvPort))
		self.sendSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)# UDP

	def sendFunction(self, msg, ip, port):
		self.sendSocket.sendto(msg, (ip, port))

	def recvFunction(self):
		data, addr = self.recvSocket.recvfrom(1024) # buffer size is 1024 bytes
		return data, addr