import gobject
import sys
import socket
import time
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from datetime import datetime
from MainGUIWidgets import * 
from GstUtil import *
from ClientProtocol import *

import thread
import threading
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *

# CLIENT_RECV_UDP_IP = "127.0.0.1"
# CLIENT_RECV_UDP_PORT = 10001
# CLIENT_SEND_UDP_IP = "127.0.0.1"
# CLIENT_SEND_UDP_PORT = 10000

# GUI_SEND_UDP_IP = "127.0.0.1"
# GUI_SEND_UDP_PORT = 12000

#UNUSED
# TELEMETRY_RECV_UDP_IP = "127.0.0.1"
# TELEMETRY_RECV_UDP_PORT = 30002

# OPERATOR GUI RECV
TELEMETRY1_SEND_UDP_IP = "127.0.0.1"
TELEMETRY1_SEND_UDP_PORT = 30001
GUI_CONTROLLER1_RECV_UDP_IP = "127.0.0.1"
GUI_CONTROLLER1_RECV_UDP_PORT = 30009
GUI_COMMS1_RECV_UDP_IP = "127.0.0.1"
GUI_COMMS1_RECV_UDP_PORT = 30010
# NAVIGATOR GUI RECV
TELEMETRY2_SEND_UDP_IP = "127.0.0.1"
TELEMETRY2_SEND_UDP_PORT = 30003
# OPERATOR GUI RECV (MSI)
TELEMETRY3_SEND_UDP_IP = "192.168.1.104"
TELEMETRY3_SEND_UDP_PORT = 30001
GUI_CONTROLLER3_RECV_UDP_IP = "192.168.1.104"
GUI_CONTROLLER3_RECV_UDP_PORT = 30009
GUI_COMMS3_RECV_UDP_IP = "192.168.1.104"
GUI_COMMS3_RECV_UDP_PORT = 30010
# NAVIGATOR GUI RECV (MSI)
TELEMETRY4_SEND_UDP_IP = "192.168.1.104"
TELEMETRY4_SEND_UDP_PORT = 30003

THIS_TELEMETRY_SEND_UDP_IP = TELEMETRY1_SEND_UDP_IP
THIS_TELEMETRY_SEND_UDP_PORT=TELEMETRY1_SEND_UDP_PORT
THIS_GUI_CONTROLLER_RECV_UDP_IP = GUI_CONTROLLER1_RECV_UDP_IP
THIS_GUI_CONTROLLER_RECV_UDP_PORT = GUI_CONTROLLER1_RECV_UDP_PORT
THIS_GUI_COMMS_RECV_UDP_IP = GUI_COMMS1_RECV_UDP_IP
THIS_GUI_COMMS_RECV_UDP_PORT = GUI_COMMS1_RECV_UDP_PORT

MCAST_GRP = '224.1.1.2'
MCAST_PORT = 5007
MCAST_RECV_PORT = 10240 # IDK WHY

THREAD_DELAY = 0.01

class OperatorGUI(QMainWindow):
	Signal_OperatorGUI_Quit = pyqtSignal() #signal to send for closing

	def __init__(self):
		super(OperatorGUI, self).__init__()
		self.setWindowTitle('OperatorGUI')
		self.setToolTip('This is a <b>QWidget</b> widget - GUI Base')
		self.GUISize = QSize(1280,720)
		self.resize(self.GUISize.width(),self.GUISize.height())
		center(self)
		self._initUI()
		# 5. bind inputs to widgets
		#self.roverThread = RoverTelemetryThread()
		self.clientThread = ClientNetworkThread()
		self.client = ClientComms(self)
		self.widgetSignals = self.GUITelemetryInterface(self)
		#self.roverThreadConnect(self.roverThread, self.widgetSignals)
		self.clientThreadConnect(self.clientThread, self.widgetSignals)
		self.clientConnect(self.client, self.widgetSignals)
		self.client.startThreads()
		self.clientThread.start()
		#self.roverThread.start()
		self.expandMinimap = False

	def _initUI(self):
		# 1. build pipeline
		self.NUMBER_CAMERAS = 1
		self.pipelines = GStreamerPipeline()
		self.pipelines.startup(self.NUMBER_CAMERAS)

		# 2. build widgets
		self.videoDisplayWidget = VideoDisplayWidget(self.GUISize.width(), self.GUISize.height(), self.pipelines, self)
		self.pipelines.Trigger_New_Frame().connect(self.videoDisplayWidget.Slot_Trigger_New_Frame)
		self.compassWidget = CompassWidget(350,30, 5, 15, self)
		self.driveBarWidget = DriveBarWidget(200,50, 10,10,20,5,20, self)
		self.scienceBarWidget = DriveBarWidget(200,50, 10,10,20,5,20, self)
		self.taskListWidget = TaskListWidget(1, 3,12,5, 12, 10, self)
		self.crosshairWidget = CrosshairWidget(20,20, self)
		self.roverTiltWidget = RoverTiltWidget(100,100, 2, self)
		self.roverTiltWidget.setVisible(False)
		self.HUDWidget = HUDOverlayWidget(self.GUISize.width(), self.GUISize.height(), 45, 10, self)
		self.signalWidget = SignalQualityWidget(200,30, 10, self)
		self.signalWidget.setVisible(False)
		self.minimapWidget = MinimapWidget(200,200, 10, self)
		self.camStatusWidget = CameraStatusWidget(50,20, self)
		self.gpsDisplayWidget = GPSDisplayWidget(200,30,12, self)
		self.camStatusWidget.Signal_Camera_Set.emit('Main')
		self.camStatusWidget.Signal_MenuCamera_Set.emit(str(self.videoDisplayWidget._cameraIndex))
		self.helpWidget = HelpWidget(500,500, 12, self)
		self.helpMenuBuild(self.helpWidget)
		# 3. move widgets
		self.videoDisplayWidget.move(0,0)
		self.compassWidget.move(self.GUISize.width()/2 - self.compassWidget.width/2, 0)
		self.driveBarWidget.move(self.GUISize.width()-self.driveBarWidget.width, self.GUISize.height()-self.driveBarWidget.height)
		self.scienceBarWidget.move(self.GUISize.width()-self.scienceBarWidget.width, self.GUISize.height()-self.scienceBarWidget.height-self.driveBarWidget.height)
		self.crosshairWidget.move(self.GUISize.width()/2 - self.crosshairWidget.width/2, self.GUISize.height()/2 - self.crosshairWidget.height/2)
		self.roverTiltWidget.move(self.GUISize.width()/2, self.GUISize.height()-self.roverTiltWidget.height)
		self.HUDWidget.move(0,0)
		# self.signalWidget.move(self.GUISize.width()-self.signalWidget.width, self.GUISize.height()-self.driveBarWidget.height-self.signalWidget.height)
		self.minimapWidget.move(0, self.GUISize.height()-self.minimapWidget.height)
		self.camStatusWidget.move(0,0)
		self.taskListWidget.move(0,self.camStatusWidget.height)
		self.helpWidget.move(self.GUISize.width()/2 - self.helpWidget.width/2, self.GUISize.height()/2 - self.helpWidget.height/2)
		self.gpsDisplayWidget.move(0, self.GUISize.height()-self.minimapWidget.height-self.gpsDisplayWidget.height)
		# self.gpsDisplayWidget.setVisible(True)
		# 4. load test cases
		self.loadTestCases()
		print 'start index', self.videoDisplayWidget._cameraIndex

	def loadTestCases(self):
		# 4. build test tasks
		self.taskListWidget.Signal_Add_Task.emit("0.", "Header", 0, False)
		self.taskListWidget.Signal_Add_Task.emit("1.", "test!", 0, False)
		self.taskListWidget.Signal_Add_Task.emit("2.", "test!", 0, False)
		self.taskListWidget.Signal_Add_Task.emit("3.", "test!", 0, False)
		# 5. build test checkpoints
		# self.HUDWidget.Signal_Insert_Nav_Point.emit(0, 10, "Pos1")
		self.HUDWidget.Signal_Move_Camera_X.emit(180)# 0-360 0=back,180=front
		self.HUDWidget.Signal_Move_Camera_Y.emit(0)# -fov-+fov <left-right>
		
	def keyPressEvent(self, e):
		if e.key() == Qt.Key_Escape: # Esc
			self.close()
		elif e.key() == Qt.Key_G:
			if not self.expandMinimap:
				self.minimapWidget.Signal_Set_Size.emit(500,500)
				self.minimapWidget.move(640-250,360-250)
				self.repaint()
				self.expandMinimap = True
			else:
				self.minimapWidget.Signal_Set_Size.emit(200,200)
				self.minimapWidget.move(0, self.GUISize.height()-self.minimapWidget.height)
				self.minimapWidget.repaint()
				self.expandMinimap = False
		elif e.key() == Qt.Key_F1:
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, gst.STATE_PLAYING)
		elif e.key() == Qt.Key_F2:
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, gst.STATE_PAUSED)
		elif e.key() == Qt.Key_F3:
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, gst.STATE_NULL)
		elif e.key() == Qt.Key_F5:# Prev Camera
			if self.videoDisplayWidget._cameraIndex == 0:
				self.videoDisplayWidget._cameraIndex = self.NUMBER_CAMERAS-1
			else:
				self.videoDisplayWidget._cameraIndex -= 1
			state = self.camStatusWidget.currentVideoStatus
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, state)
		elif e.key() == Qt.Key_F6:# Next Camera
			if self.videoDisplayWidget._cameraIndex == self.NUMBER_CAMERAS-1:
				self.videoDisplayWidget._cameraIndex = 0
			else:
				self.videoDisplayWidget._cameraIndex += 1
			# print 'next cam',self.videoDisplayWidget._cameraIndex
			state = self.camStatusWidget.currentVideoStatus
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, state)
		elif e.key() == Qt.Key_BracketLeft:
			self.taskListWidget.Signal_Task_Not_Done.emit(0)
		elif e.key() == Qt.Key_BracketRight:
			self.taskListWidget.Signal_Task_Done.emit(0)
		elif e.key() == Qt.Key_Backslash:
			self.taskListWidget.Signal_Set_Phase.emit(-1)
		elif e.key() == Qt.Key_Apostrophe:
			self.crosshairWidget.Signal_Crosshair_Type.emit((self.crosshairWidget._crosshair_type+1)%2)
		elif e.key() == Qt.Key_Semicolon:
			self.minimapWidget.Signal_Update_Map_Type.emit((self.minimapWidget._Map_Types+1)%len(self.minimapWidget._topoQuads))
		elif e.key() == Qt.Key_Slash:
			self.helpWidget.setVisible(not self.helpWidget.isVisible())

	def changeCameraStatus(self, index, num):
		self.videoDisplayWidget.currentVideoStatus = num
		self.pipelines.Signal_Set_Camera().emit(index, num)
		self.camStatusWidget.Signal_Camera_Status.emit(num)
		self.camStatusWidget.Signal_MenuCamera_Set.emit(str(index))

	def closeEvent(self, event):
		#self.screenVideo.streamer.quit()
		self.client.endThreads()
		self.clientThread.isRunning = False
		self.clientThread.sock.sendto("end",(self.clientThread.UDP_IP,self.clientThread.UDP_PORT))
		self.clientThread.sock.close()
		event.accept()
		sys.exit()

	def helpMenuBuild(self, widget):
		widget.addHelpText("Shortcuts:")
		widget.addHelpText("F1 : Play Main Camera")
		widget.addHelpText("F2 : Pause Main Camera")
		widget.addHelpText("F3 : Stop Main Camera")
		widget.addHelpText("\\  : hide HUD")
		widget.addHelpText("G  : expand Map")
		widget.addHelpText("M  : change map")
		widget.addHelpText("N  : science heading mode")
		widget.addHelpText("-  : zoom out map")
		widget.addHelpText("+  : zoom in  map")
		widget.addHelpText("[: undo task")
		widget.addHelpText("]: done task")
		widget.addHelpText("Tab: change task list display(1/2/3)")
		widget.addHelpText("/  : This help menu")
		widget.addHelpText("Esc: Exit GUI")

	def clientConnect(self, client, interface):
		client.Signal_Drive_Volt_Recv.connect(interface.Slot_Drive_Volt_Recv)
		client.Signal_Drive_Volt_Send.connect(interface.Slot_Drive_Volt_Send)
		client.Signal_Science_Volt_Recv.connect(interface.Slot_Science_Volt_Recv)
		client.Signal_Science_Volt_Send.connect(interface.Slot_Science_Volt_Send)
		client.Signal_ICCE_TiltX.connect(interface.Slot_ICCE_TiltX)
		client.Signal_ICCE_TiltY.connect(interface.Slot_ICCE_TiltY)
		client.Signal_ICCE_Compass.connect(interface.Slot_ICCE_Compass)
		client.Signal_ICCE_GPS.connect(interface.Slot_ICCE_GPS)

	def clientThreadConnect(self, clientThread, interface):
		clientThread.Signal_Client_Timer.connect(interface.Slot_Server_Timer)
		clientThread.Signal_Client_Navpt.connect(interface.Slot_Client_Navpt)
		clientThread.Signal_Client_Navrt.connect(interface.Slot_Client_Navrt)
		clientThread.Signal_Client_Task.connect(interface.Slot_Client_Task)

	# def roverThreadConnect(self, roverThread, interface):
	# 	roverThread.Signal_ICCE_Compass.connect(interface.Slot_ICCE_Compass)
	# 	roverThread.Signal_ICCE_GPS.connect(interface.Slot_ICCE_GPS)
	# 	roverThread.Signal_ICCE_TiltX.connect(interface.Slot_ICCE_TiltX)
	# 	roverThread.Signal_ICCE_TiltY.connect(interface.Slot_ICCE_TiltY)
	# 	roverThread.Signal_Drive_Volt_Send.connect(interface.Slot_Drive_Volt_Send)
	# 	roverThread.Signal_Drive_Volt_Recv.connect(interface.Slot_Drive_Volt_Recv)
	# 	roverThread.Signal_Camera_Pan.connect(interface.Slot_Camera_Pan)
	# 	roverThread.Signal_Camera_Tilt.connect(interface.Slot_Camera_Tilt)
	# 	roverThread.Signal_Server_Timer.connect(interface.Slot_Server_Timer)
	# 	roverThread.Signal_Server_Link_Quality.connect(interface.Slot_Server_Link_Quality)
	# 	roverThread.Signal_Server_Signal_Strength.connect(interface.Slot_Server_Signal_Strength)
	# 	roverThread.Signal_Server_Ping.connect(interface.Slot_Server_Ping)

	class GUITelemetryInterface():
		def __init__(self, parent=None):
			self.parent = parent

		def Slot_ICCE_Compass(self, heading):
			print 'update heading',heading
			self.parent.compassWidget.Signal_Update_Heading.emit(heading)
			self.parent.minimapWidget.Signal_Update_Rover_Heading.emit(heading)
			self.parent.HUDWidget.Signal_Move_Camera_X.emit((heading+180)%360)
		def Slot_ICCE_GPS(self, lat, lon, alt):
			if lat == -1:
				lat = self.parent.minimapWidget._rover_longitude
			if lon == -1:
				lon = self.parent.minimapWidget._rover_latitude
			self.parent.HUDWidget.Signal_Update_Rover_GPS.emit(lat, lon)
			self.parent.minimapWidget.Signal_Update_Rover_GPS.emit(lat, lon)
			self.parent.minimapWidget.Signal_View_GPS.emit(lat, lon)
			self.parent.gpsDisplayWidget.Signal_Update_GPS.emit(lat,lon,alt)
		def Slot_ICCE_TiltX(self, tiltX):
			self.parent.roverTiltWidget.Signal_Update_TiltX.emit(tiltX)
		def Slot_ICCE_TiltY(self, tiltY):
			self.parent.roverTiltWidget.Signal_Update_TiltY.emit(tiltY)
		def Slot_Drive_Volt_Send(self, motorNum, percentVoltage):
			self.parent.driveBarWidget.Signal_Update_SentSpeed.emit(motorNum, percentVoltage)
		def Slot_Drive_Volt_Recv(self, motorNum, percentVoltage):
			self.parent.driveBarWidget.Signal_Update_Speed.emit(motorNum, percentVoltage)
		def Slot_Science_Volt_Send(self, motorNum, percentVoltage):
			self.parent.scienceBarWidget.Signal_Update_SentSpeed.emit(motorNum, percentVoltage)
		def Slot_Science_Volt_Recv(self, motorNum, percentVoltage):
			self.parent.scienceBarWidget.Signal_Update_Speed.emit(motorNum, percentVoltage)
		def Slot_Camera_Pan(self, heading):
			self.parent.roverTiltWidget.Signal_Rotate_Base.emit(heading)
			self.parent.HUDWidget.Signal_Move_Camera_X.emit((heading+180)%360)
			self.parent.minimapWidget.Signal_Update_Camera_Heading.emit(0, heading)
		def Slot_Camera_Tilt(self, tilt):
			self.parent.roverTiltWidget.Signal_Rotate_Angle.emit(tilt)
			self.parent.HUDWidget.Signal_Move_Camera_Y.emit(tilt)
			self.parent.minimapWidget.Signal_Update_Camera_Angle.emit(0, tilt)
		def Slot_Server_Timer(self, time):
			self.parent.taskListWidget.Signal_Update_Time.emit(time)
		def Slot_Server_Link_Quality(self, lq):
			self.parent.signalWidget.Signal_Update_Link_Quality.emit(lq)
		def Slot_Server_Signal_Strength(self, ss):
			self.parent.signalWidget.Signal_Update_Signal_Strength.emit(ss)
		def Slot_Server_Ping(self, ping):
			self.parent.signalWidget.Signal_Update_Ping.emit(ping)
		def Slot_Client_Navpt(self, x,y, tag):
			if tag == 'CLEAR':
				self.parent.minimapWidget.Signal_Clear_Nav_Point.emit()
				self.parent.HUDWidget.Signal_Clear_Nav_Point.emit()
			else:
				self.parent.minimapWidget.Signal_Nav_Point.emit(x,y,tag)
				self.parent.HUDWidget.Signal_Insert_Nav_Point.emit(x,y, tag)
		def Slot_Client_Navrt(self, x,y, x2,y2):
			if x == y == x2 == y2 == -1:
				self.parent.minimapWidget.Signal_Clear_Nav_Route.emit()
			else:
				self.parent.minimapWidget.Signal_Nav_Route.emit(x,y, x2,y2)
		def Slot_Client_Task(self, tag,msg,time,done):
			if tag == msg == 'clear':
				self.parent.taskListWidget.Signal_Clear_Tasks.emit()
			elif tag == msg == 'undo':
				self.parent.taskListWidget.Signal_Task_Not_Done.emit(0)
			elif tag == msg == 'done':
				self.parent.taskListWidget.Signal_Task_Done.emit(0)
			else:
				self.parent.taskListWidget.Signal_Add_Task.emit(tag,msg,time,not done)

class ClientNetworkThread(QThread):
	Signal_Client_Timer = pyqtSignal(int)
	Signal_Client_Navpt = pyqtSignal(float,float, QString)
	Signal_Client_Navrt = pyqtSignal(float,float,float,float)
	Signal_Client_Task = pyqtSignal(str,str,float,bool)

	def __init__(self, parent=None):
		super(ClientNetworkThread, self).__init__(parent)
		self.parent = parent
		self.isRunning = True
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		self.UDP_IP = THIS_GUI_COMMS_RECV_UDP_IP
		self.UDP_PORT = THIS_GUI_COMMS_RECV_UDP_PORT
		self.sock.bind((self.UDP_IP,self.UDP_PORT))
		self.protocol = ClientProtocol()
		
	def run(self):
		print 'start thread'
		while self.isRunning:
			data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
			print "received message:", data
			if data.startswith(self.protocol.PROTOCOL_TIME):
				self.Signal_Client_Timer.emit(self.protocol.decodeTimeMsg(data))
			elif data.startswith(self.protocol.PROTOCOL_NAVPT):
				arr = self.protocol.decodeNavPointMsg(data)# X,Y, 
				print 'arr',arr
				print 'data',data
				if arr == 'clear':
					self.Signal_Client_Navpt.emit(-1,-1,'CLEAR')
				elif len(arr) == 1:# one; append
					self.Signal_Client_Navpt.emit(float(arr[0][0]),float(arr[0][1]),arr[0][2])
				# elif len(arr) > 1:# refresh; clear all, reload
				# 	self.Signal_Client_Navpt.emit(-1,-1,'CLEAR')
				# 	for i in range(len(arr)):
				# 		self.Signal_Client_Navpt.emit(float(arr[i][0]),float(arr[i][1]),arr[i][2])
			elif data.startswith(self.protocol.PROTOCOL_NAVRT):
				arr = self.protocol.decodeNavRouteMsg(data)
				if arr == 'clear':
					print 'clear'
					self.Signal_Client_Navrt.emit(-1,-1,-1,-1)
				if len(arr) == 1:
					self.Signal_Client_Navrt.emit(float(arr[0][0]),float(arr[0][1]),float(arr[0][2]),float(arr[0][3]))
				# elif len(arr) > 1:
				# 	self.Signal_Client_Navrt.emit(-1,-1,-1,-1)
				# 	for i in range(len(arr)):
				# 		self.Signal_Client_Navrt.emit(float(arr[0][0]),float(arr[0][1]),float(arr[0][2]),float(arr[0][3]))
			elif data.startswith(self.protocol.PROTOCOL_TASK):
				print type(data),data
				ret, arr = self.protocol.decodeTaskListMsg(data)
				print ret,arr
				if ret.startswith('refresh='):
					self.Signal_Client_Task.emit('clear','clear',-1,False)
					for i in range(len(arr)):
						self.Signal_Client_Task.emit(arr[i][0],arr[i][1],float(arr[i][2]),bool(arr[i][3]))
				elif ret.startswith('undo'):
					self.Signal_Client_Task.emit('undo','undo',-1,False)
				elif ret.startswith('done'):
					self.Signal_Client_Task.emit('done','done',-1,False)
				elif ret.startswith('add='):
					for i in range(len(arr)):
						self.Signal_Client_Task.emit(arr[i][0],arr[i][1],arr[i][2],arr[i][3])

class ClientComms(QThread):
	Signal_ICCE_Compass = pyqtSignal(int)
	Signal_ICCE_GPS = pyqtSignal(float, float, float)
	Signal_ICCE_TiltX = pyqtSignal(float)
	Signal_ICCE_TiltY = pyqtSignal(float)
	Signal_Drive_Volt_Send = pyqtSignal(int, float) # DONE
	Signal_Drive_Volt_Recv = pyqtSignal(int, float) # DONE
	Signal_Science_Volt_Send = pyqtSignal(int, float) # DONE
	Signal_Science_Volt_Recv = pyqtSignal(int, float) # DONE
	# Signal_Camera_Pan = pyqtSignal(float)
	# Signal_Camera_Tilt = pyqtSignal(float)
	# Signal_Server_Timer = pyqtSignal(int)# NOT USED
	# Signal_Server_Link_Quality = pyqtSignal(str)
	# Signal_Server_Signal_Strength = pyqtSignal(str)
	# Signal_Server_Ping = pyqtSignal(str)
	#----------------       1. INIT COMMANDS        ----------------#
	def __init__(self, parent=None):
		super(ClientComms, self).__init__(parent)
		self.parent = parent
		self.rover = RoverModel()
		self._client_TelemetryComms = UniCastComms(THIS_TELEMETRY_SEND_UDP_IP,THIS_TELEMETRY_SEND_UDP_PORT)
		self._client_CommandComms = UniCastComms(THIS_GUI_CONTROLLER_RECV_UDP_IP, THIS_GUI_CONTROLLER_RECV_UDP_PORT)
		self.runThreads = True
		self.client_MultiCastThread = RecvThread(self, 1, self._Server_MultiCast_RecvFunction, self._Server_MultiCast_ProcessTelemetry, THREAD_DELAY)
		self.client_UniCastThread = RecvThread(self, 2, self._Server_UniCast_RecvFunction, self._Server_UnitCast_RecvCommandSent, THREAD_DELAY)
		
	def startThreads(self):
		self.client_MultiCastThread.start()
		self.client_UniCastThread.start()
		
	def endThreads(self):
		self.runThreads = False
		# self._Server_MultiCast_SendFunction('end')
		self._client_TelemetryComms.sendFunction('end',THIS_TELEMETRY_SEND_UDP_IP,THIS_TELEMETRY_SEND_UDP_PORT)
		self._Server_UniCast_SendFunction('end', THIS_GUI_CONTROLLER_RECV_UDP_IP, THIS_GUI_CONTROLLER_RECV_UDP_PORT)
		
	#----------------       2. CLIENT MULTICAST FUNCTIONS       ----------------#
	def _Server_MultiCast_RecvFunction(self):
		data, addr = self._client_TelemetryComms.recvFunction()
		return data

	def _Server_MultiCast_SendFunction(self, msg):
		self._client_TelemetryComms.sendFunction(msg,TELEMETRY_RECV_UDP_IP,TELEMETRY_RECV_UDP_PORT)
		# self._client_TelemetryComms.sendFunction(msg,THIS_TELEMETRY_SEND_UDP_IP,THIS_TELEMETRY_SEND_UDP_PORT)

	def _Server_MultiCast_ProcessTelemetry(self, num, msg):
		print 'thread(multi)',num,' got:',msg
		if msg == 'end':
			'sentEnd'
			self.runThreads = False
			self._client_TelemetryComms.close()
			self._client_CommandComms.close()
		elif self.rover.updateTelemetry(msg):
			path, value, chksum = self.rover.decode(msg)
			print 'update!',path,value,chksum
			if path == NS_MOTOR_VOLT:
				spi = value.split(',')
				self.Signal_Drive_Volt_Recv.emit(1, float(spi[0]) * -1 * -1)
				self.Signal_Drive_Volt_Recv.emit(0, float(spi[1]) * -1)
				self.Signal_Drive_Volt_Recv.emit(3, float(spi[2]) * -1)
				self.Signal_Drive_Volt_Recv.emit(2, float(spi[3]) * -1)
			elif path == NS_MOTORFL_VOLT:
				self.Signal_Drive_Volt_Recv.emit(0, float(value))
			elif path == NS_MOTORFR_VOLT:
				self.Signal_Drive_Volt_Recv.emit(1, float(value))
			elif path == NS_MOTORRL_VOLT:
				self.Signal_Drive_Volt_Recv.emit(2, float(value))
			elif path == NS_MOTORRR_VOLT:
				self.Signal_Drive_Volt_Recv.emit(3, float(value))
			elif path == NS_IMU_X:
				self.Signal_ICCE_TiltX.emit(float(value))
			elif path == NS_IMU_Y:
				self.Signal_ICCE_TiltY.emit(float(value))
			elif path == NS_COMPASS_HEADING or path == NS_COMPASS:
				print 'got heading'
				# 243 = South; so 290-x=180,x=110
				self.Signal_ICCE_Compass.emit((int(value)-110)%360)
			elif path == NS_GPS:
				spi = value.split(',')
				self.Signal_ICCE_GPS.emit(float(spi[1]), float(spi[0]), float(spi[2]))
				print 'gps:',spi
			elif path == NS_GPS_LATI:
				self.Signal_ICCE_GPS.emit(float(value), -1, -1)
			elif path == NS_GPS_LONG:
				self.Signal_ICCE_GPS.emit(-1, float(value), -1)
			elif path == NS_DRILL:
				self.Signal_Science_Volt_Recv.emit(1, float(value))
				self.Signal_Science_Volt_Recv.emit(3, float(value))
				pass
			elif path == NS_CORE:
				self.Signal_Science_Volt_Recv.emit(0, float(value))
				self.Signal_Science_Volt_Recv.emit(2, float(value))
				pass

	def _Server_MultiCast_TestFunction(self, num, msg):
		print 'thread(multi)',num,' got:',msg

	#----------------       3. CLIENT UNICAST FUNCTIONS     ----------------#
	def _Server_UniCast_SendFunction(self, msg, ip, port):
		self._client_CommandComms.sendFunction(msg, ip, port)

	def _Server_UniCast_RecvFunction(self):
		data,addr = self._client_CommandComms.recvFunction()
		return data

	# CONTROLLER RECV
	def _Server_UnitCast_RecvCommandSent(self, num, msg):
		# Controller
		print 'thread(uni)',num,' got:',msg
		path, value,chksum = self.rover.decode(msg)
		if path == NS_MOTOR_VOLT:
			spi = value.split(',')
			# FL = FR
			# FR = FL
			# RL = RR
			# RR = RL
			self.Signal_Drive_Volt_Send.emit(1, float(spi[0]) * -1 * -1)
			self.Signal_Drive_Volt_Send.emit(0, float(spi[1]) * -1)
			self.Signal_Drive_Volt_Send.emit(3, float(spi[2]) * -1)
			self.Signal_Drive_Volt_Send.emit(2, float(spi[3]) * -1)
		elif path == NS_MOTORFL_VOLT:
			self.Signal_Drive_Volt_Send.emit(0, float(value))
		elif path == NS_MOTORFR_VOLT:
			self.Signal_Drive_Volt_Send.emit(1, float(value))
		elif path == NS_MOTORRL_VOLT:
			self.Signal_Drive_Volt_Send.emit(2, float(value))
		elif path == NS_MOTORRR_VOLT:
			self.Signal_Drive_Volt_Send.emit(3, float(value))
			# pass
		elif path == NS_DRILL:
			self.Signal_Science_Volt_Send.emit(1, float(value))
			self.Signal_Science_Volt_Send.emit(3, float(value))
			pass
		elif path == NS_CORE:
			self.Signal_Science_Volt_Send.emit(0, float(value))
			self.Signal_Science_Volt_Send.emit(2, float(value))
			pass

	def _Server_UniCast_TestFunction(self, num, msg):
		print 'thread( uni )',num,' got:',msg

class RoverTelemetryThread(QThread):
	Signal_ICCE_Compass = pyqtSignal(int)
	Signal_ICCE_GPS = pyqtSignal(float, float)
	Signal_ICCE_TiltX = pyqtSignal(float)
	Signal_ICCE_TiltY = pyqtSignal(float)
	Signal_Drive_Volt_Send = pyqtSignal(int, float)
	Signal_Drive_Volt_Recv = pyqtSignal(int, float)
	Signal_Camera_Pan = pyqtSignal(float)
	Signal_Camera_Tilt = pyqtSignal(float)
	Signal_Server_Timer = pyqtSignal(int)
	Signal_Server_Link_Quality = pyqtSignal(str)
	Signal_Server_Signal_Strength = pyqtSignal(str)
	Signal_Server_Ping = pyqtSignal(str)

	def __init__(self, parent=None):
		super(RoverTelemetryThread, self).__init__(parent)
		self.parent = parent
		self.isRunning = True
		self.compassHeading = 0
		self.timer = 3600
		start = 0
		self.driveSend = [start,start,start,start]
		self.driveRecv = [start,start,start,start]
		self.roverGPS  = [-110.813180853,38.4375030973]
		self.roverTilt = [0,0]
		self.camOrientation = [0,0]# pan[0,360],tilt[-90,90]
		self.signal = [0,0,0]# link quality, signal level, ping
		
		
	def run(self):
		print 'start thread'
		self.setICCE_GPS()
		while self.isRunning:
			# self.updateTestParams()
			# self.setICCE_Compass()
			# self.setICCE_Timer()
			# self.setDrive_Send()
			# self.setDrive_Recv()
			# self.setICCE_Tilt()
			# self.setCamera_Ori()
			# self.setSignal()
			time.sleep(1)
			# print "compass:"+str(self.compassHeading)
			# print "timer:"+str(self.timer)
			# print "driveSend:"+str(self.driveSend)
			# print "driveRecv:"+str(self.driveRecv)
		pass

	def updateTestParams(self):
		if self.compassHeading > 360:
			self.compassHeading = 0
		else:
			self.compassHeading += 1
		if self.timer > 0:
			self.timer -= 1
		if self.driveSend[0] < 100:
			for i in range(len(self.driveSend)):
				self.driveRecv[i] = self.driveSend[i]
				self.driveSend[i] += 1
		else:
			for i in range(len(self.driveSend)):
				self.driveRecv[i] = -100
				self.driveSend[i] = -100
		if self.roverTilt[0] > 90:
			self.roverTilt[0] = -90
			self.roverTilt[1] = -90
		else:
			self.roverTilt[0] += 1
			self.roverTilt[1] += 1
		if self.camOrientation[0] > 360:
			self.camOrientation[0] = 0
		else:
			self.camOrientation[0] += 1
		if self.camOrientation[1] > 90:
			self.camOrientation[1] = -90
		else:
			self.camOrientation[1] += 1
		if self.signal[0] > 90:
			for i in range(len(self.signal)):
				self.signal[i] = 0
		else:
			for i in range(len(self.signal)):
				self.signal[i] += 1

	def setICCE_Compass(self):
		self.Signal_ICCE_Compass.emit(self.compassHeading)
		
	def setICCE_Timer(self):
		self.Signal_Server_Timer.emit(self.timer)

	def setICCE_GPS(self):
		self.Signal_ICCE_GPS.emit(self.roverGPS[0], self.roverGPS[1])

	def setDrive_Send(self):
		for i in range(len(self.driveSend)):
			self.Signal_Drive_Volt_Send.emit(i, self.driveSend[i])

	def setDrive_Recv(self):
		for i in range(len(self.driveSend)):
			self.Signal_Drive_Volt_Recv.emit(i, self.driveRecv[i])

	def setICCE_Tilt(self):
		self.Signal_ICCE_TiltX.emit(self.roverTilt[0])
		self.Signal_ICCE_TiltY.emit(self.roverTilt[1])

	def setCamera_Ori(self):
		self.Signal_Camera_Pan.emit(self.camOrientation[0])
		self.Signal_Camera_Tilt.emit(self.camOrientation[1])

	def setSignal(self):
		self.Signal_Server_Link_Quality.emit(str(self.signal[0])+'/70')
		self.Signal_Server_Signal_Strength.emit(str(self.signal[1]))
		self.Signal_Server_Ping.emit(str(self.signal[2]))
'''
Centers the main window on the screen.
'''
def center(widget):
	qr = widget.frameGeometry()
	cp = QDesktopWidget().availableGeometry().center()
	qr.moveCenter(cp)
	widget.move(qr.topLeft())

def main():
	app = QApplication(sys.argv)	# application object for PyQT
	gui = OperatorGUI()
	gui.show()
	loop = gobject.MainLoop()
	try:
		loop.run()
		app.exec_()
	except KeyboardInterrupt:
		print "Ctrl+C pressed, exitting"
		gui.client.endThreads()
		sys.exit()
		pass

if __name__ == '__main__':
	ret = gst.element_register(VideoSink, 'VideoSink')
	gobject.threads_init()
	main()
'''
RECORD
gst-launch-1.0 -v -e v4l2src device=/dev/video1 num-buffers=500 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4
'''
'''
MAIN
gst-launch-1.0 -v -e v4l2src device=/dev/video1 num-buffers=500 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t \
  ! queue ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=1234 \
  t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4

gst-launch-1.0 -v -e udpsrc port=1234 ! multiudpsink clients="127.0.0.1:5000,127.0.0.1:5001"
gst-launch-1.0 -v -e udpsrc port=5001 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
'''
'''
SSH LEVEL WORKS
gst-launch -v -e v4l2src device=/dev/video0 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t ! queue ! rtph264pay ! udpsink host=192.168.7.1 port=1234 
t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4

gst-launch-1.0 -v -e udpsrc port=1234 ! multiudpsink clients="127.0.0.1:5000,127.0.0.1:5001"
gst-launch-1.0 -v -e udpsrc port=5001 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
'''
'''
MAIN
gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t ! queue ! rtph264pay ! udpsink host=192.168.11.6 port=1234
  t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4

gst-launch-1.0 -v -e udpsrc port=1234 ! multiudpsink clients="127.0.0.1:5000,127.0.0.1:5001"
gst-launch-1.0 -v -e udpsrc port=5001 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
'''
'''
XBEE TEST
gst-launch-1.0 -v -e v4l2src device=/dev/video1 num-buffers=500 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t ! queue ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=1234
  t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4

gst-launch-1.0 -v -e udpsrc port=1234 ! multiudpsink clients="127.0.0.1:5000,127.0.0.1:5001"
gst-launch-1.0 -v -e udpsrc port=5001 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
'''