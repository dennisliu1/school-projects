#Communications to the rover. This is the only code that actually talks to the rover. 
#All communications goes through this.

# Server Multicast = pushing Telemetry to all clients (one way only)
# Server UDP unicast = commands and special operations (two way)
'''
sudo xboxdrv --detach-kernel-driver
'''
import math
import sys
import thread
import time
import threading
import socket
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *
from GamepadClient import *

pygame.init()

# SEND CMDS TO SERVERCOMM
SERVER_RECV_UDP_IP = "192.168.1.103"
SERVER_RECV_UDP_PORT = 20000
# RECV CMDS FROM SERVERCOMM
CLIENT_RECV_UDP_IP = "192.168.1.103"
CLIENT_RECV_UDP_PORT = 20001
GUI_COMMS1_RECV_UDP_IP = "192.168.1.103"
GUI_COMMS1_RECV_UDP_PORT = 30010
# GUI_SEND_UDP_IP = "192.168.1.103"
# GUI_SEND_UDP_PORT = 12000
# OPERATOR GUI SEND
TELEMETRY1_SEND_UDP_IP = "192.168.1.103"
TELEMETRY1_SEND_UDP_PORT = 30001
GUI_CONTROLLER1_RECV_UDP_IP = "192.168.1.103"
GUI_CONTROLLER1_RECV_UDP_PORT = 30009
GUI_CONTROLLER3_RECV_UDP_IP = "192.168.1.104"
GUI_CONTROLLER3_RECV_UDP_PORT = 30009
# NAVIGATOR GUI SEND
TELEMETRY2_SEND_UDP_IP = "192.168.1.103"
TELEMETRY2_SEND_UDP_PORT = 30003

# MCAST_GRP = '224.1.1.2'
# MCAST_PORT = 5007
# MCAST_RECV_PORT = 10240 # IDK WHY

THREAD_DELAY = 0.01

class ClientComms:
	#----------------       1. INIT COMMANDS        ----------------#
	def __init__(self,ctrl=0,gamepad=0):
		self.rover = RoverModel()
		self.count = 0
		self.ctrl = ctrl
		# self._client_TelemetryComms = MultiCastComms(MCAST_GRP, MCAST_PORT, MCAST_RECV_PORT)
		self._client_CommandComms = UniCastComms(CLIENT_RECV_UDP_IP, CLIENT_RECV_UDP_PORT)
		self.runThreads = True
		# self.client_MultiCastThread = RecvThread(self, 1, self._Server_MultiCast_RecvFunction, self._Server_MultiCast_TestFunction, THREAD_DELAY)
		self.client_UniCastThread = RecvThread(self, 2, self._Server_UniCast_RecvFunction, self._Server_UniCast_TestFunction, THREAD_DELAY)
		self.sendThread = self.SendThread(self, 3, self._SendCommand, self._closeSendThread, 0.1)
		self.clients = [
			[GUI_CONTROLLER1_RECV_UDP_IP,GUI_CONTROLLER1_RECV_UDP_PORT],
			[GUI_CONTROLLER3_RECV_UDP_IP,GUI_CONTROLLER3_RECV_UDP_PORT]
		]
		if ctrl == 0:
			self.gamepad = self.GamepadRover(self,gamepad)
		elif ctrl == 1:
			self.gamepad = self.GamepadRoverARM(self,gamepad)
		else:
			self.gamepad = self.GamepadRover(self,gamepad)
		self.gamepad.startThreads()
		
	def startThreads(self):
		# self.client_MultiCastThread.start()
		self.client_UniCastThread.start()
		self.sendThread.start()
		
	def endThreads(self):
		self.runThreads = False
		# self._Server_MultiCast_SendFunction('end')
		self._Server_UniCast_SendFunction('end', CLIENT_RECV_UDP_IP, CLIENT_RECV_UDP_PORT)
		self.gamepad.isRunning = False
		self.sendThread.isRunning = False
		
	#----------------       2. CLIENT MULTICAST FUNCTIONS       ----------------#
	# def _Server_MultiCast_RecvFunction(self):
	# 	return self._client_TelemetryComms.recvFunction()

	# def _Server_MultiCast_SendFunction(self, msg):
	# 	self._client_TelemetryComms.sendFunction(msg)

	# def _Server_MultiCast_TestFunction(self, num, msg):
	# 	print 'thread(multi)',num,' got:',msg

	#----------------       3. CLIENT UNICAST FUNCTIONS     ----------------#
	def _Server_UniCast_SendFunction(self, msg, ip, port):
		self._client_CommandComms.sendFunction(msg, ip, port)

	def _Server_UniCast_RecvFunction(self):
		data,addr = self._client_CommandComms.recvFunction()
		return data

	def _Server_UniCast_TestFunction(self, num, msg):
		print 'thread( uni )',num,' got:',msg

	#----------------       3. SEND THREAD FUNCTIONS     ----------------#

	def send(self, msg):
		# if self.rover.updateTelemetry(msg):
		print 'message','::'+msg+'::'
		self._Server_UniCast_SendFunction(msg, SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
		for i in range(len(self.clients)):
			self._Server_UniCast_SendFunction(msg, self.clients[i][0],self.clients[i][1])
		# self._Server_UniCast_SendFunction(msg, GUI_SEND_UDP_IP, GUI_SEND_UDP_PORT)

	# send rover commands
	def _SendCommand(self, num):
		if self.count%2 == 1:
			if self.ctrl == 0:
				# DRIVE
				arr = self.rover.get(NS_MOTOR_VOLT)
				val = str(int(arr.child[0]))+N_SEP_VAL+str(int(arr.child[1]))+N_SEP_VAL+str(int(arr.child[2]))+N_SEP_VAL+str(int(arr.child[3]))
				msg = self.rover.encode(NS_MOTOR_VOLT, val)
				self.send(msg)
				# # SCIENCE CORE
				# arr = self.rover.get(NS_CORE)
				# val = str(int(arr.child[0]))
				# msg = self.rover.encode(NS_CORE, val)
				# self.send(msg)
				# # SCIENCE DRILL
				# arr = self.rover.get(NS_DRILL)
				# val = str(int(arr.child[0]))
				# msg = self.rover.encode(NS_DRILL, val)
				# self.send(msg)
			if self.ctrl == 1:
				# if not self.gamepad.armMode:
				# DRIVE
				arr = self.rover.get(NS_MOTOR_VOLT)
				val = str(int(arr.child[0]))+N_SEP_VAL+str(int(arr.child[1]))+N_SEP_VAL+str(int(arr.child[2]))+N_SEP_VAL+str(int(arr.child[3]))
				msg = self.rover.encode(NS_MOTOR_VOLT, val)
				self.send(msg)

		# if self.count%1 == 0:
		# 	if self.gamepad.armMode:
		# 		self.gamepad.sendArmEndMsg()
		# 		self.gamepad.sendArmMainMsg()
		
		if self.count > 10:
			self.count = 0
		else:
			self.count += 1
		pass

	def _closeSendThread(self):
		pass

	class SendThread(threading.Thread):
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

	#----------------       AUXILLARY FUNCTIONS     ----------------#
	class GamepadRover(InputManager):
		def __init__(self, parent,gamepad=0):
			InputManager.__init__(self,gamepad)
			self.parent = parent
			self.left = 0
			self.right = 0
			self.leftTrig = False
			self.rightTrig = False
			self.driveButton = False
			self.isScience = False
			self.isLocked = True

		def buttonUpEvent(self, num, button):
			print 'buttonUp',num,button
			if num == 4 and button == 0:
				self.driveButton = False
				# mFL = 0 #forward right
				# mFR = 0 #forward left
				# mRL = 0 #backward right
				# mRR = 0 #forward left
				# val = str(mFL)+N_SEP_VAL+str(mFR)+N_SEP_VAL+str(mRL)+N_SEP_VAL+str(mRR)
				# msg = self.parent.rover.encode(NS_MOTOR_VOLT, val)
				# # print 'msg',msg
				# self.send(msg)
			pass

		def buttonDownEvent(self, num, button):
			print 'buttonDown',num,button
			if num == 0 and button == 1:# Circle
				self.isScience = False
				self.parent._Server_UniCast_SendFunction('drive', GUI_COMMS1_RECV_UDP_IP, GUI_COMMS1_RECV_UDP_PORT)
				# self.send(msg)
				# self.parent.rover.updateTelemetry(msg)
			elif num == 1 and button == 1:# X
				self.isScience = True
				self.parent._Server_UniCast_SendFunction('science', GUI_COMMS1_RECV_UDP_IP, GUI_COMMS1_RECV_UDP_PORT)

			# self.send('button_'+str(num)+','+str(button)+'E')
			# if num == 0 and button == 1:
			# 	msg = self.parent.rover.encode(NS_TRAP, str(N_TRAP_OC_OPEN))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif num == 1 and button == 1:
			# 	msg = self.parent.rover.encode(NS_TRAP, str(N_TRAP_OC_CLOSED))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# if num == 2 and button == 1:
			# 	msg = self.parent.rover.encode(NS_SPECT, str(N_SPECT_IO_IN))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif num == 3 and button == 1:
			# 	msg = self.parent.rover.encode(NS_SPECT, str(N_SPECT_IO_OUT))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif num == 4 and button == 1:
			# 	self.driveButton = True
			# elif num == 5 and button == 1:#hat right?
			# 	pass
			# elif num == 6 and button == 1:#hat down
			# 	pass
			elif num == 8 and button == 1:#hat down
				self.isLocked = True
			elif num == 9 and button == 1:#hat down
				self.isLocked = False
				# self.isRunning = False
				# self.parent.endThreads()
				# sys.exit()
			pass

		def controlFunction(self, axis):
			if axis != 0:
				result=math.pow(axis,2)*100
				result *= axis/abs(axis)
				return result
			else:
				return 0

		# PS3 CONTROLLER  C10_-100,-100,100,-100E
		def axisEvent2(self, axis):
			if self.isLocked:
				return
			# print 'axis:', axis1, axis2, axis3, axis4
			# X2 DRIVE
			# self.left  = -1 * int(self.controlFunction(axis[1]))
			# self.right = -1 * int(self.controlFunction(axis[3]))

			# LINEAR DRIVE
			self.left  = -1 * int(axis[1] * 100)
			self.right = -1 * int(axis[3] * 100)
			# ratio1 = 0
			# ratio2 = 0
			# deadzone = 0.3
			# if -deadzone < axis[1] < deadzone:
			# 	axis[1] = 0
			# else:
			# 	ratio1 = (axis[1]-deadzone)/(1-deadzone)
			# if -deadzone < axis[3] < deadzone:
			# 	axis[3] = 0
			# else:
			# 	ratio2 = (axis[3]-deadzone)/(1-deadzone)
			# self.left  = -1 * int(ratio1 * 100)
			# self.right = -1 * int(ratio2 * 100)
			# if -deadzone < self.left < deadzone:
			# 	self.left = 0
			# if -deadzone < self.right < deadzone:
			# 	self.right = 0
			
			if not self.isScience:
				# print 'lr', self.left, self.right
				mFL = self.right #forward right DONE
				mFR = -1 * self.left #forward left DONE
				mRL = -1 * self.right#backward right DONE
				mRR = -1 * self.left#forward left DONE
				val = str(mFL)+N_SEP_VAL+str(mFR)+N_SEP_VAL+str(mRL)+N_SEP_VAL+str(mRR)
				msg = self.parent.rover.encode(NS_MOTOR_VOLT, val)
				self.parent.rover.updateTelemetry(msg)
			else:
				drill_speed = self.left
				msg = self.parent.rover.encode(NS_CORE, str(drill_speed))
				self.parent.rover.updateTelemetry(msg)
				core_speed = self.right
				msg = self.parent.rover.encode(NS_DRILL, str(core_speed))
				self.parent.rover.updateTelemetry(msg)
			# print 'msg',msg
			# self.send(msg)
			# self.isRunning = False
		# 	# self.parent.endThreads()

		def axisEvent(self, num, axis):
			pass

		def sendCmd(self):
			# if (self.leftTrig and self.rightTrig):
			# 	val = str(self.left)+','+str(self.right)+','+str(self.left)+','+str(self.right)
			# 	msg = self.parent.rover.encode('C', NS_MOTOR_VOLT, val)
			# 	self.send(msg)
				# self.left = 0
				# self.right = 0
				# self.leftTrig = False
				# self.rightTrig = False
			# self.isRunning = False
			# self.parent.endThreads()
			pass

		def send(self, msg):
			print 'message','::'+msg+'::'
			if self.parent.rover.updateTelemetry(msg):
				self.parent._Server_UniCast_SendFunction(msg, SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
			for i in range(len(self.parent.clients)):
				self.parent._Server_UniCast_SendFunction(msg, self.parent.clients[i][0],self.parent.clients[i][1])
				# self.parent._Server_UniCast_SendFunction(msg, GUI_SEND_UDP_IP, GUI_SEND_UDP_PORT)

		def ballEvent(self, num, ball):
			print 'ball:', num, axis
			pass

		def hatEvent(self, num, hat):
			if self.isLocked:
				return
			print 'hat:', num, hat#, dir(hat)
			# self.send('button_'+str(num)+','+str(hat)+'E')
			# lr,ud = hat
			# if lr == 1:# up
			# 	msg = self.parent.rover.encode(NS_CORE, str(100))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif lr == 0:
			# 	msg = self.parent.rover.encode(NS_CORE, str(0))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif lr == -1:
			# 	msg = self.parent.rover.encode(NS_CORE, str(-100))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# if ud == 1:
			# 	msg = self.parent.rover.encode(NS_DRILL, str(100))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif ud == 0:
			# 	msg = self.parent.rover.encode(NS_DRILL, str(0))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			# elif ud == -1:
			# 	msg = self.parent.rover.encode(NS_DRILL, str(-100))
			# 	# self.send(msg)
			# 	self.parent.rover.updateTelemetry(msg)
			pass

		# def buttonUpEvent(self, num, button):
		# 	print 'bUp:', num, button
		# 	pass

		# def buttonDownEvent(self, num, button):
		# 	print 'bDown:', num, button
		# 	# self.parent.endThreads()
		# 	pass

	class GamepadRoverARM(InputManager):
		def __init__(self, parent,gamepad):
			InputManager.__init__(self,gamepad)
			print 'gamepadRover init'
			self.parent = parent
			self.armMainX = 0# Base[-1,1]
			self.armMainY = 0# Act1[-100,100]
			self.armMainZ = 0# Act2[-100,100]
			self.armEndX = 0
			self.armEndY = 0
			self.armEndZ = 0
			self.armEndRotate = 0
			self.armEndGrip = 0
			self.armEndYaw = 0
			self.leftTrig = False
			self.rightTrig = False
			self.driveButton = False
			self.armMode = False
			self.isLocked = True

		def buttonUpEvent(self, num, button):
			print 'buttonUp',num,button
			# if num == 4 and button == 0:
			# 	self.driveButton = False
			if num == 0 and button == 0:
				self.armEndRotate = -0
				# self.sendArmEndMsg()
			elif num == 1 and button == 0:
				self.armEndRotate = 0
				# self.sendArmEndMsg()
			elif num == 2 and button == 0:
				self.armEndGrip = 0
				# self.sendArmEndMsg()
			elif num == 3 and button == 0:
				self.armEndGrip = 0
				# self.sendArmEndMsg()
			elif num == 4 and button == 0:# L1
				self.armMainX = 0
				# self.sendArmMainMsg()
			elif num == 5 and button == 0:# R1
				# self.armEndZ = 0
				self.armEndYaw = 0
				# self.sendArmEndMsg()
			elif num == 6 and button == 0:# L2
				self.armMainX = 0
				# self.sendArmMainMsg()
			elif num == 7 and button == 0:# R2
				# self.armEndZ = 0
				self.armEndYaw = 0
				# self.sendArmEndMsg()
			pass

		def buttonDownEvent(self, num, button):
			print 'buttonDown',num,button
			# self.send('button_'+str(num)+','+str(button)+'E')
			if num == 8 and button == 1:# Back
				self.isLocked = True
			elif num == 9 and button == 1:# Start
				self.isLocked = False
			if self.isLocked:
				return

			if num == 0 and button == 1:
				self.armEndRotate = -1
				# self.sendArmEndMsg()
			elif num == 1 and button == 1:
				self.armEndRotate = 1
				# self.sendArmEndMsg()

			elif num == 2 and button == 1:
				self.armEndGrip = 1
				# self.sendArmEndMsg()
			elif num == 3 and button == 1:
				self.armEndGrip = -1
				# self.sendArmEndMsg()

			elif num == 4 and button == 1:# L1
				self.armMainX = 1
				# self.sendArmMainMsg()
			elif num == 5 and button == 1:# R1
				# self.armEndZ = 1
				self.armEndYaw = 1
				# self.sendArmEndMsg()
			elif num == 6 and button == 1:# L2
				self.armMainX = -1
				# self.sendArmMainMsg()
			elif num == 7 and button == 1:# R2
				# self.armEndZ = -1
				self.armEndYaw = -1
				# self.sendArmEndMsg()
				# self.isRunning = False
				# self.parent.endThreads()

			# self.isRunning = False
			# self.parent.endThreads()
			pass

		def controlFunction(self, axis):
			if axis != 0:
				result=math.pow(axis,2)*100
				result *= axis/abs(axis)
				return result
			else:
				return 0


		# PS3 CONTROLLER  C10_-100,-100,100,-100E
		def axisEvent2(self, axis):
			print 'axis:', axis
			if self.isLocked:
				return
			if self.armMode:
				# BASE ARM
				if axis[0] >= 0.99:
					axis[0] = 1
				if axis[1] >= 0.99:
					axis[1] = 1
				self.armMainZ = -1 * int(axis[0] * 100)# Up/Down
				self.armMainY = -1 * int(axis[1] * 100)# L/R
				# if -5 < self.armMainY < 5:
				# 	self.armMainY = 0
				# if -5 < self.armMainX < 5:
				# 	self.armMainX = 0
				# if -5 < self.armMainZ < 5:
				# 	self.armMainZ = 0
				# self.sendArmMainMsg()
				
				# END EFFECTOR
				if axis[2] >= 0.99:
					axis[2] = 1
				if axis[3] >= 0.99:
					axis[3] = 1
				self.armEndZ = -1 * int(axis[2] * 100)# U/D
				self.armEndY = -1 * int(axis[3] * 100)# L/R
				# if -5 < self.armEndY < 5:
				# 	self.armEndY = 0
				# if -5 < self.armEndX < 5:
				# 	self.armEndX = 0
				# if -5 < self.armEndZ < 5:
				# 	self.armEndZ = 0
				# self.sendArmEndMsg()
			else:
				# LINEAR DRIVE
				self.left  = -1 * int(axis[1] * 100)
				self.right = -1 * int(axis[3] * 100)
				if -5 < self.left < 5:
					self.left = 0
				if -5 < self.right < 5:
					self.right = 0
				# print 'lr', self.left, self.right
				mFL = self.right #forward right DONE
				mFR = -1 * self.left #forward left DONE
				mRL = -1 * self.right#backward right DONE
				mRR = -1 * self.left#forward left DONE
				val = str(mFL)+N_SEP_VAL+str(mFR)+N_SEP_VAL+str(mRL)+N_SEP_VAL+str(mRR)
				msg = self.parent.rover.encode(NS_MOTOR_VOLT, val)
				self.parent.rover.updateTelemetry(msg)
				
			# self.isRunning = False
			# self.parent.endThreads()

		# def axisEvent(self, num, axis):
		# 	pass

		def sendCmd(self):
			pass

		def send(self, msg):
			print 'message','::'+msg+'::'
			# if self.rover.updateTelemetry(msg):
			print 'message','::'+msg+'::'
			self.parent._Server_UniCast_SendFunction(msg, SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
			for i in range(len(self.parent.clients)):
				self.parent._Server_UniCast_SendFunction(msg, self.parent.clients[i][0],self.parent.clients[i][1])
			# if self.parent.rover.updateTelemetry(msg):
			# 	self.parent._Server_UniCast_SendFunction(msg, SERVER_RECV_UDP_IP, SERVER_RECV_UDP_PORT)
			# 	self.parent._Server_UniCast_SendFunction(msg, GUI_SEND_UDP_IP, GUI_SEND_UDP_PORT)

		def ballEvent(self, num, ball):
			print 'ball:', num, axis
			pass

		def hatEvent(self, num, hat):
			print 'hat:', num, hat#, dir(hat)
			# # self.send('button_'+str(num)+','+str(hat)+'E')
			# lr,ud = hat
			# if lr == 1:# up
			# 	self.armMode = False
			# 	print 'armMode',self.armMode
			# elif lr == 0:
			# 	pass
			# elif lr == -1:
			# 	self.armMode = True
			# 	print 'armMode',self.armMode
			# if ud == 1:
			# 	self.armEndYaw = 1
			# elif ud == 0:
			# 	self.armEndYaw = 0
			# elif ud == -1:
			# 	self.armEndYaw = -1
			pass

		def sendArmMainMsg(self):
			val = str(self.armMainX)+N_SEP_VAL+str(self.armMainY)+N_SEP_VAL+str(self.armMainZ)
			msg = self.parent.rover.encode(NS_ARM_MAIN, val)
			self.send(msg)

		def sendArmEndMsg(self):
			val = str(self.armEndX)+N_SEP_VAL+str(self.armEndY)+N_SEP_VAL+str(self.armEndZ)+N_SEP_VAL+str(self.armEndGrip)+N_SEP_VAL+str(self.armEndRotate)+N_SEP_VAL+str(self.armEndYaw)
			msg = self.parent.rover.encode(NS_ARM_END, val)
			self.send(msg)

def main(ctrl=0,gamepad=0):
	print "ClientComms main"
	client = ClientComms(ctrl,gamepad)
	client.startThreads()
	while True:
		try:
			time.sleep(1)
			if not client.runThreads:
				break
		except KeyboardInterrupt:
			print "Ctrl+C pressed, exitting"
			client.endThreads()
			break
			pass

if __name__ == "__main__":
	main(int(sys.argv[1]),int(sys.argv[2]))