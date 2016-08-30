#!/usr/bin/python
# -*- coding: utf-8 -*-
#if master navigator:
	# push changes to local copy
	# changes are sent down nav_pt_mirror,nav_route_mirror
	# whenever a slave sends an empty message down nav_pt_refresh,
		# master navigator sends a copy of the current nav_pt/nav_route.
		# Use nav_pt_refresh,nav_route_refresh
# rosrun gpsd_client gpsd_client _host:=localhost _port:=2947 _use_gps_time:=false
# rostopic echo /fix
# midpt = -110.813125109,38.4374592146
'''
:-110.773800815,38.3833597702
:-110.774122169,38.3834780312
:-110.774253872,38.3834925545
:-110.774240702,38.3834199381
:-110.774296017,38.3834635079
:-110.774296017,38.3834635079
:-110.774296017,38.3834635079
:-110.774335528,38.3834510594
:-110.774335528,38.3834323866
:-110.774340796,38.3834614332
:-110.774338162,38.3835029283
:-110.774359234,38.3834137138
:-110.77436977,38.3834510594
:-110.774290749,38.3834157886
:-110.77413534,38.3834178633
:-110.774087927,38.3834261623
:-110.774108999,38.3834074895
:-110.77403788,38.3834137138
:-110.774003637,38.3833888168
:-110.773950956,38.3834012653
:-110.773950956,38.3834012653
:-110.773858764,38.3833991905
:-110.773821887,38.3833971158
:-110.773748134,38.3833825925
:-110.773624334,38.3833369479
:-110.773613797,38.3833597702
:-110.773476827,38.3833431722
:-110.773429414,38.3833079013
:-110.773358294,38.38327678
:-110.773326686,38.3832456587
:-110.773276639,38.3833037518
:-110.773189715,38.3833493964
:-110.773142302,38.3833514712
:-110.773047476,38.3834427604
:-110.772944748,38.3833908915
:-110.772907871,38.3834614332
:-110.772381061,38.3834303118
:-110.77254964,38.3835299001
:-110.772402133,38.3835776194
:-110.772288869,38.3836377873
:-110.772225652,38.383741525
:-110.772346818,38.3837664221
:-110.772444278,38.3839386267
:-110.77245218,38.3839884208
:-110.772583883,38.3842871854
:-110.772886799,38.3843743251
:-110.772794607,38.3845008852
:-110.772691879,38.3844842871
:-110.77296582,38.3846191462
:-110.77296582,38.3846191462
:-110.771999123,38.384679314
:-110.773073816,38.3846378189
:-110.773144936,38.3846876131
:-110.773181813,38.3846689403
:-110.773292443,38.3844905114
:-110.773353026,38.3843826241
:-110.773379367,38.3842892602
:-110.773384635,38.3842207933
:-110.773318783,38.3841232798
:-110.773123863,38.3840340654
:-110.773123863,38.3840340654
:-110.773123863,38.3840340654
:-110.773521605,38.3840817848
:-110.773555848,38.3839738975
:-110.773529508,38.3839095801
:-110.773666478,38.3839012811
:-110.773698087,38.3838494122
:-110.773260834,38.3836938057
:-110.773766572,38.3838037677
:-110.773985199,38.383606666
:-110.77394042,38.3835236758
:-110.773893007,38.3834863302
:-110.77403788,38.3833722187
:-110.774111633,38.3834178633
:-110.773882471,38.383260182
:-110.774180119,38.3833079013
:-110.774267042,38.383268481
:-110.774338162,38.3832145374
:-110.774396111,38.3831834161
:-110.77442772,38.3831232482
:-110.774504107,38.383056856
:-110.774535716,38.3830278095
:-110.774651614,38.3828929504
:-110.774677955,38.3828493806
:-110.77475961,38.3827456429
:-110.774706929,38.3826522789
:-110.774612103,38.3824074579
:-110.774754342,38.3821875339
'''
'''
Border Quadrants
:-110.805053332,38.3964596333||:-110.786783134,38.3965013219
:-110.805000375,38.4110089335||:-110.805053332,38.3964596333
:-110.786624263,38.4110506221||:-110.805000375,38.4110089335
:-110.786783134,38.3965430104||:-110.786624263,38.4110506221
:-110.786470966,38.4110725634||:-110.768195193,38.4110506221
:-110.7865072,38.42536515||:-110.786470966,38.4110725634
:-110.768094853,38.4255231276||:-110.7865072,38.42536515
:-110.768220278,38.4110879223||:-110.768094853,38.4255231276

DEM:
High = 1538.22
Low = 1320.82

---:Task List Widget
01:Start Rover
02:Goto site 1
03:Goto site 2
04:Goto site 3
05:Goto site 4
06:Goto site 5
'''

import sys
import socket
import time
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from MainGUIWidgets import *
from ClientProtocol import *
from datetime import datetime

import thread
import threading
import struct
from ..Shared.RoverModel import *
from ..Shared.RoverComms import *

# CLIENT_RECV_UDP_IP = "127.0.0.1"
# CLIENT_RECV_UDP_PORT = 10001
# CLIENT_SEND_UDP_IP = "127.0.0.1"
# CLIENT_SEND_UDP_PORT = 10000

# OPERATOR GUI RECV
TELEMETRY1_SEND_UDP_IP = "127.0.0.1"
TELEMETRY1_SEND_UDP_PORT = 30001
GUI_COMMS1_RECV_UDP_IP = "127.0.0.1"
GUI_COMMS1_RECV_UDP_PORT = 30010
# NAVIGATOR GUI RECV
TELEMETRY2_SEND_UDP_IP = "127.0.0.1"
TELEMETRY2_SEND_UDP_PORT = 30003
# OPERATOR GUI RECV (MSI)
TELEMETRY3_SEND_UDP_IP = "192.168.1.104"
TELEMETRY3_SEND_UDP_PORT = 30001
GUI_COMMS3_RECV_UDP_IP = "192.168.1.104"
GUI_COMMS3_RECV_UDP_PORT = 30010
# NAVIGATOR GUI RECV (MSI)
TELEMETRY4_SEND_UDP_IP = "127.168.1.104"
TELEMETRY4_SEND_UDP_PORT = 30003

SERVERCOMMS_UDP_IP = "127.0.0.1"
SERVERCOMMS_UDP_PORT=10000

# MCAST_GRP = '224.1.1.2'
# MCAST_PORT = 5007
# MCAST_RECV_PORT = 10240 # IDK WHY

THREAD_DELAY = 0.01

class NavigatorGUI(QMainWindow):
	Signal_Update_Time = pyqtSignal(int)
	def __init__(self, width,height):
		super(NavigatorGUI, self).__init__()
		self.setWindowTitle('NavigatorGUI')
		self.setWindowIcon(QIcon('hp_cat.png'))
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.Signal_Update_Time.connect(self._Slot_Update_Time)

		self.rover = RoverModel()
		self.startMoveX = 0
		self.startMoveY = 0
		self.endMoveX = 0
		self.endMoveY = 0
		self.isLeftButton = False
		self.inputMode = 0#0=point 1=route
		self.prevRouteEnd = [0.0,0.0]
		self.map_type = 0#0=topo 1=aerial
		self.accessType = 1 # 0:read,1:offline,2:write
		self.isFollowingRover = True
		# self.ROSThread = ROSControlThread(self)
		# self.ROSThread.Signal_ALES_GPS.connect(self._Slot_ALES_GPS)
		self.UDP_IP = "127.0.0.1"
		self.UDP_PORT = 25005
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		self.protocol = ClientProtocol()
		self.rPixel = [53,30]
		self.clients = [
			[GUI_COMMS1_RECV_UDP_IP,GUI_COMMS1_RECV_UDP_PORT],
			[GUI_COMMS3_RECV_UDP_IP,GUI_COMMS3_RECV_UDP_PORT]
		]

		self.initUI()
		self.setFocus()

		self.runThreads = True
		self.isExecRoute = False
		self.client = self.ClientComms(self,self.rover)
		self.navcomThread = self.NAVCOMThread(self,1,self.NAVCOM_ProcessFunct,self.NAVCOM_CloseFunct, 0.2)#def __init__(self, parent, num, recvFunc, threadFunc, delay):
		self.widgetSignals = self.GUITelemetryInterface(self)
		self.clientConnect(self.client, self.widgetSignals)
		self.client.startThreads()
		self.navcomThread.start()

		msg = self.rover.encode(NS_CLEAR, '1')
		self.client._Server_UniCast_SendFunction(msg, SERVERCOMMS_UDP_IP, SERVERCOMMS_UDP_PORT)
		self.navcomThread.endRoutes()

	def initUI(self):
		self.splitter = QSplitter(Qt.Horizontal, self)
		self.splitter.resize(self.width,self.height)
		#self.splitter.setFrameShape(QFrame.StyledPanel)
		self.boxWidget = QWidget(self)
		self.mapBoxWidget = QWidget(self)
		self.helpWidget = HelpWidget(500,500, 12, self)
		self.helpMenuBuild(self.helpWidget)

		roverRadius = 5
		self.mapWidget = MinimapWidget(self.width,self.height, roverRadius, self.mapBoxWidget)
		self.mapWidget.move(0,0)
		self._mapMid = [self.mapWidget._midCoord[0],self.mapWidget._midCoord[1]]
		#self.mapWidget._PIXEL_COORD

		self.tasklistWidget_ratio = [4.5,3.0]
		width = self.rPixel[0]*self.tasklistWidget_ratio[0]
		height = self.rPixel[1]*self.tasklistWidget_ratio[1]
		margin = width/10.0
		phase1 = 5
		phase2 = 15
		phase3 = 5
		numRows= 10
		fontLen= (height/numRows)
		self.taskWidget = TaskListWidget(margin,phase1,phase2,phase3,fontLen,numRows, self)
		self.taskWidget.move(0,0)
		self.GPSDisplayWidget_ratio = [8.0,1.0]
		width = self.rPixel[0]*self.GPSDisplayWidget_ratio[0]
		height = self.rPixel[1]*self.GPSDisplayWidget_ratio[1]
		font = 10
		self.GPSDpWidget = GPSDisplayWidget(width,height,font, self)
		self.GPSDpWidget.move(0,self.height-height)
		self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1], 0)
		
		self.masterSwitch_ratio = [8.0,1.0]
		width = self.rPixel[0]*self.masterSwitch_ratio[0]
		height= self.rPixel[1]*self.masterSwitch_ratio[1]
		self.masterSwitch = self.DataControlWidget(width,height, self,self.boxWidget)
		self.masterSwitch.move(10,0)
		self.navPointListWidget_ratio = [8.0,7.0]
		width = self.rPixel[0]*self.navPointListWidget_ratio[0]
		height= self.rPixel[1]*self.navPointListWidget_ratio[1]
		self.navPointListWidget = self.NavPointListWidget(width,height, self,self.boxWidget)
		self.navPointListWidget.move(10,self.masterSwitch.height)
		self.navRouteListWidget_ratio = [8.0,7.0]
		width = self.rPixel[0]*self.navRouteListWidget_ratio[0]
		height= self.rPixel[1]*self.navRouteListWidget_ratio[1]
		self.navRouteListWidget = self.NavRouteListWidget(width,height, self,self.boxWidget)
		self.navRouteListWidget.move(10,self.masterSwitch.height+self.navPointListWidget.height)
		self.taskListWidget_ratio = [8.0,6.0]
		width = self.rPixel[0]*self.taskListWidget_ratio[0]
		height= self.rPixel[1]*self.taskListWidget_ratio[1]
		self.taskListWidget = self.EditTaskListWidget(width,height, self,self.boxWidget)
		self.taskListWidget.move(10,self.masterSwitch.height+self.navPointListWidget.height+5+self.navRouteListWidget.height)

		self.timeWidget_ratio = [8.0,1.0]
		width = self.rPixel[0]*self.timeWidget_ratio[0]
		height= self.rPixel[1]*self.timeWidget_ratio[1]
		self.timeWidget = self.TimerControlWidget(width,height, self,self.boxWidget)
		self.timeWidget.move(10,self.masterSwitch.height+self.navPointListWidget.height+5+self.navRouteListWidget.height+5+self.taskListWidget.height)
		self.timeWidget.Signal_Update_Time.connect(self._Slot_Update_Time)
		self.inputLabel = QLabel("Point Follow", self.boxWidget)#space is to make default size show "Route" properly
		self.inputLabel.move(10,self.masterSwitch.height+self.navPointListWidget.height+5+self.navRouteListWidget.height+5+self.taskListWidget.height+5+self.timeWidget.height)
		#self.taskWidget.current_task

		self.splitter.addWidget(self.mapWidget)
		self.splitter.addWidget(self.boxWidget)

	def _Slot_Update_Time(self, time):
		self.taskWidget.Signal_Update_Time.emit(time)
		self.taskWidget.repaint()
		self.timeWidget.time = time
		self.timeWidget.setTime()
		if self.accessType == 2:
			msg = self.protocol.encodeTimeMsg(time)
			self.sendMsg(msg)
			#self.ROSThread.pub_BASE_Timer.publish(time)

	def _Slot_ALES_GPS(self, longitude, latitude):
		self.mapWidget.Signal_Update_Rover_GPS.emit(longitude,latitude,0)
		self.GPSDpWidget.Signal_Update_GPS.emit(longitude,latitude,0)

	def helpMenuBuild(self, widget):
		widget.addHelpText("Shortcuts:")
		widget.addHelpText("Esc: Exit GUI")
		widget.addHelpText("/  : This help menu")
		widget.addHelpText("F12: Take Screenshot")
		widget.addHelpText("WAQE  : Direction move")
		widget.addHelpText("-/=  : Zoom in/out")
		widget.addHelpText("N  : toggle point/route")
		widget.addHelpText("M  : switch maps")
		widget.addHelpText("X  : undo route")
		widget.addHelpText("Z  : undo point")

	def closeEvent(self, event):
		self.runThreads = False
		self.client.endThreads()
		self.navcomThread.isRunning = False
		self.sock.close()
		event.accept()
		sys.exit()

	def keyPressEvent(self, e):
		if e.key() == Qt.Key_Escape: # Esc
			self.close()
		elif e.key() == Qt.Key_Slash:
			self.helpWidget.setVisible(not self.helpWidget.isVisible())
		elif e.key() ==  Qt.Key_F12:
			date = datetime.now()
			filename = date.strftime('%Y-%m-%d_%H-%M-%s.jpg')
			picture = QPixmap.grabWindow(self.winId())
			picture.save(filename, 'jpg')
			print 'took picture!'
		elif e.key() == Qt.Key_W:#Up
			#self.mapWidget._Map_Types
			self._mapMid[1] += self.mapWidget._PIXEL_COORD[0][1] #0.00002194137265#math.sin(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_Y
			self.mapWidget.Signal_View_GPS.emit(self._mapMid[0],self._mapMid[1])
			#self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
			
		elif e.key() == Qt.Key_S:#Down
			self._mapMid[1] -= self.mapWidget._PIXEL_COORD[0][1] #0.00002194137265#*math.sin(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_Y
			self.mapWidget.Signal_View_GPS.emit(self._mapMid[0],self._mapMid[1])
			#self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
			
		elif e.key() == Qt.Key_Q:#Left Strafe
			self._mapMid[0] -= self.mapWidget._PIXEL_COORD[0][0] #0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.mapWidget.Signal_View_GPS.emit(self._mapMid[0],self._mapMid[1])
			#self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
			
		elif e.key() == Qt.Key_E:#Right Strafe
			self._mapMid[0] += self.mapWidget._PIXEL_COORD[0][0] #0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.mapWidget.Signal_View_GPS.emit(self._mapMid[0],self._mapMid[1])
			#self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
		elif e.key() ==  Qt.Key_Equal:# plus;zoom in
			self.mapWidget.zoomIncrease(0.1)
		elif e.key() ==  Qt.Key_Minus:# minus;zoom out
			self.mapWidget.zoomDecrease(0.1)
		elif e.key() ==  Qt.Key_N:
			msg = ""
			if self.inputMode == 1:
				self.inputMode = 0
				self.prevRouteEnd = [0.0,0.0]
				msg += "Point"
			else:
				self.inputMode = 1
				msg += "Route"
			if self.isFollowingRover:
				msg += " Follow"
			else:
				msg += " Stay"
			self.inputLabel.setText(msg)
			print 'inputMode:',self.inputMode
		elif e.key() == Qt.Key_Semicolon:
			if self.map_type < len(self.mapWidget._topoQuads)-1:
				self.map_type += 1
			elif self.map_type == len(self.mapWidget._topoQuads)-1:
				self.map_type = 0
			self.mapWidget.Signal_Update_Map_Type.emit(self.map_type)
		elif e.key() == Qt.Key_Z:
			if self.accessType >= 1:
				self.mapWidget.popNavPoint()
				self.mapWidget.repaint()
				self.navPointListWidget.button_refreshNavPoints()
		elif e.key() == Qt.Key_X:
			if self.accessType >= 1:
				self.prevRouteEnd[0] = 0.0
				self.prevRouteEnd[1] = 0.0
				self.mapWidget.popNavRoute()
				self.mapWidget.repaint()
				self.navRouteListWidget.button_refreshNavRoutes()
		elif e.key() == Qt.Key_F:
			self.isFollowingRover = not self.isFollowingRover
			print 'inputMode',self.inputMode
			msg = ""
			if self.inputMode == 0:
				msg = "Point"
			else:
				msg = "Route"
			if self.isFollowingRover:
				msg += " Follow"
			else:
				msg += " Stay"
			self.inputLabel.setText(msg)
	
	def wheelEvent(self, event):
		distance = event.delta()
		if distance > 0:
			self.mapWidget.zoomDecrease(0.1)
		elif distance < 0:
			self.mapWidget.zoomIncrease(0.1)
		#print 'scroll:',distance
	def mouseDoubleClickEvent(self, event):
		#print 'double click',event.x(),' ',event.y()
		pass
	def mouseMoveEvent(self, event):
		#print 'move ',event.x(),' ',event.y()
		if self.isLeftButton:
			self.endMoveX = event.x()
			self.endMoveY = event.y()
			moveX = (self.startMoveX - self.endMoveX)*self.mapWidget._zoom
			moveY = (self.startMoveY - self.endMoveY)*self.mapWidget._zoom
			resultX = self._mapMid[0]+moveX*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][0]#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			resultY = self._mapMid[1]+-1*moveY*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][1]#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.mapWidget.Signal_View_GPS.emit(resultX,resultY)
			#self.GPSDpWidget.Signal_Update_GPS.emit(resultX,resultY)
			self.repaint()
		pass
	def mousePressEvent(self, event):
		#print 'press',event.x(),' ',event.y()
		if event.button() == Qt.LeftButton:
			self.isLeftButton = True
			self.startMoveX = event.x()
			self.startMoveY = event.y()
		pass
	def mouseReleaseEvent(self, event):
		#print 'release ',event.x(),' ',event.y()
		if event.button() == Qt.LeftButton:
			self.endMoveX = event.x()
			self.endMoveY = event.y()
			moveX = (self.startMoveX - self.endMoveX)*self.mapWidget._zoom
			moveY = (self.startMoveY - self.endMoveY)*self.mapWidget._zoom
			self._mapMid[0] += moveX*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][0]#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self._mapMid[1] += -1*moveY*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][1]#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.mapWidget.Signal_View_GPS.emit(self._mapMid[0],self._mapMid[1])
			#self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
			self.isLeftButton = False
		if event.button() == Qt.RightButton:
			posX = (self.mapWidget.width/2 - event.x())*self.mapWidget._zoom
			posY = (self.mapWidget.height/2 - event.y())*self.mapWidget._zoom
			navX = self.mapWidget._midCoord[0] + -1*(posX*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][0])
			navY = self.mapWidget._midCoord[1] + posY*self.mapWidget._PIXEL_COORD[self.mapWidget._Map_Types][1]
			if self.inputMode == 0 and self.accessType >= 1:
				self.mapWidget.Signal_Nav_Point.emit(navX,navY,'')
				self.navPointListWidget.refreshNavPoints()
				if self.accessType == 2:#send pt added to other clients;mirror
					msg = self.protocol.encodeNavPointMsg([navX,navY,''])
					self.sendMsg(msg)
					#self.ROSThread.pub_BASE_Navpt_mirror.publish(Navpt(navX,navY,''))
				#print 'NavPt:',navX,' ',navY
			elif self.inputMode == 1 and self.accessType >= 1:
				if self.prevRouteEnd[0] == 0.0 and self.prevRouteEnd[1] == 0.0:
					self.prevRouteEnd = [navX,navY]
				else:
					self.mapWidget.Signal_Nav_Route.emit(navX,navY, self.prevRouteEnd[0],self.prevRouteEnd[1])
					if self.accessType == 2:#send route added to other clients; mirror
						msg = self.protocol.encodeNavRouteMsg([[navX, navY], [self.prevRouteEnd[0],self.prevRouteEnd[1]]])
						self.sendMsg(msg)
						#self.ROSThread.pub_BASE_Navroute_mirror.publish(Navroute(Navpt(navX,navY,''),Navpt(self.prevRouteEnd[0],self.prevRouteEnd[1],'')))
					self.prevRouteEnd = [navX,navY]
					self.navRouteListWidget.refreshNavRoutes()
					self.mapWidget.repaint()
		self.repaint()
		pass

	def convertDecimal(self, inputString):
		numberParse = inputString.split('_')
		print 'numberParse:',len(numberParse)
		if len(numberParse) > 1:
			if len(numberParse) >= 2:
				degree = float(numberParse[0])
				minute = float(numberParse[1])
				if len(numberParse) == 3:
					second = float(numberParse[2])
				else:
					second = 0
				decimal = 0
				decimal += minute/60.0
				decimal += second/3600.0
				if degree >= 0:
					decimal += degree
				else:
					decimal += (degree*-1)
					decimal *= -1
				print 'dms:',degree,minute,second
				print 'con:',decimal
				return decimal
			else:
				pass
		else:
			return float(numberParse[0])

	def sendMsg(self, msg):
		for i in range(len(self.clients)):
			self.sock.sendto(msg, (self.clients[i][0],self.clients[i][1]))
		# self.sock.sendto(msg, (self.UDP_IP, self.UDP_PORT))

	class NavPointListWidget(QWidget):
		Signal_refreshList = pyqtSignal()
		def __init__(self, width,height, grandparent, parent=None):
			super(NavigatorGUI.NavPointListWidget, self).__init__(parent)
			self.parent = grandparent
			self.width = width #280
			self.height = height#150
			self.resize(self.width, self.height)
			self.pointLabel = QLabel('#:long,lati',self)
			self.pointCoords = QTextEdit(self)
			self.pointCoords.resize(self.width,self.height-20)
			self.updatePoints = QPushButton("insert Points",self)
			self.refreshPoints = QPushButton("refresh Points",self)
			self.pointCoords.setLineWrapMode(QTextEdit.NoWrap)
			self.updatePoints.clicked.connect(self.button_updateNavPoints)
			self.refreshPoints.clicked.connect(self.button_refreshNavPoints)
			self.Signal_refreshList.connect(self.refreshNavPoints)

			self.updatePoints.move(0,0)
			self.refreshPoints.move(self.updatePoints.width(),0)
			self.pointLabel.move(self.updatePoints.width()+self.refreshPoints.width(),5)
			self.pointCoords.move(0,25)
			
		def updateNavPoints(self):
			self.parent.mapWidget.Signal_Clear_Nav_Point.emit()
			lines = self.pointCoords.toPlainText().split('\n')
			for i in range(len(lines)):
				try:
					pointParser = lines[i].split(':')
					tag = pointParser[0]
					coords = pointParser[1].split(',')
					decimal = [0.0,0.0]
					print 'coords:',coords[0],coords[1]
					for i in range(len(coords)):
						decimal[i] = self.parent.convertDecimal(coords[i])
					print 'decimal:',decimal[0],decimal[1]
					self.parent.mapWidget.Signal_Nav_Point.emit(decimal[0],decimal[1],tag)
					#self.parent.mapWidget.Signal_Nav_Point.emit(float(coords[0]),float(coords[1]),tag)
					#print pointParser[0],':',pointParser[1]
				except:
					pass
			self.parent.mapWidget.repaint()

		def refreshNavPoints(self):
			self.pointCoords.clear()
			points = self.parent.mapWidget._nav_points
			textString = ''
			for i in range(len(points)):
				textString += points[i][2]+':'+str(points[i][0])+','+str(points[i][1])+'\n'
				#print points[i][0],points[i][1],points[i][2]
			self.pointCoords.setText(textString)

		def button_updateNavPoints(self):
			if self.parent.accessType >= 1:
				self.updateNavPoints()
			pass

		def button_refreshNavPoints(self):
			if self.parent.accessType >= 1:
				self.refreshNavPoints()
				if self.parent.accessType == 2:
					# navptList = []
					points = self.parent.mapWidget._nav_points
					# for i in range(len(points)):
					# 	navptList.append(Navpt(points[i][0],points[i][1],str(points[i][2])))
					msg = self.parent.protocol.encodeNavPointClearMsg()
					self.parent.sendMsg(msg)
					for i in range(len(points)):
						msg = self.parent.protocol.encodeNavPointMsg(points[i])
						self.parent.sendMsg(msg)
					#self.parent.ROSThread.pub_BASE_Navpt_refresh.publish(navptList)
					print 'navpt refresh'
			pass

	class NavRouteListWidget(QWidget):
		Signal_refreshList = pyqtSignal()
		def __init__(self, width,height, grandparent, parent=None):
			super(NavigatorGUI.NavRouteListWidget, self).__init__(parent)
			self.parent = grandparent
			self.width = width #280
			self.height = height#150
			self.resize(self.width, self.height)
			self.routeLabel = QLabel('#:long,lati||#:long,lati',self)
			self.pointCoords = QTextEdit(self)
			self.pointCoords.resize(self.width,self.height-25)
			self.insertRoutes = QPushButton("Insert",self)
			self.refreshRoutes = QPushButton("Refresh Routes",self)
			self.pushRoutes = QPushButton("Push", self)
			self.execRoutes = QPushButton("Exec", self)
			self.clearRoutes = QPushButton("Clear", self)
			self.pointCoords.setLineWrapMode(QTextEdit.NoWrap)
			self.insertRoutes.clicked.connect(self.button_updateNavRoutes)
			self.refreshRoutes.clicked.connect(self.button_refreshNavRoutes)
			self.pushRoutes.clicked.connect(self.button_pushNavRoutes)
			self.execRoutes.clicked.connect(self.button_execNavRoutes)
			self.execRoutes.setEnabled(False)
			self.clearRoutes.clicked.connect(self.button_clearNavRoutes)
			self.Signal_refreshList.connect(self.refreshNavRoutes)

			self.insertRoutes.move(0,0)
			self.refreshRoutes.move(self.insertRoutes.width()-15,0)
			self.pushRoutes.move(self.insertRoutes.width()-15+self.refreshRoutes.width(),0)
			self.execRoutes.move(self.insertRoutes.width()-15+self.refreshRoutes.width()+self.pushRoutes.width()-10,0)
			self.clearRoutes.move(self.insertRoutes.width()-15+self.refreshRoutes.width()+self.pushRoutes.width()-15+self.execRoutes.width()-10,0)
			self.routeLabel.move(self.insertRoutes.width()-15+self.refreshRoutes.width()+self.pushRoutes.width()-15+self.execRoutes.width()-10+self.clearRoutes.width(),5)
			self.pointCoords.move(0,25)

		def updateNavRoutes(self):
			self.parent.mapWidget.Signal_Clear_Nav_Route.emit()
			lines = self.pointCoords.toPlainText().split('\n')
			for i in range(len(lines)):
				try:
					decimal = [0.0,0.0]
					decimal2 = [0.0,0.0]
					lineParser = lines[i].split('||')
					pointParser = lineParser[0].split(':')
					tag = pointParser[0]
					coords = pointParser[1].split(',')
					for j in range(len(coords)):
						decimal[j] = self.parent.convertDecimal(coords[j])
					pointParser2 = lineParser[1].split(':')
					tag2 = pointParser2[0]
					coords2 = pointParser2[1].split(',')
					for j in range(len(coords2)):
						decimal2[j] = self.parent.convertDecimal(coords2[j])
					print str(decimal[0]),str(decimal[1]),str(decimal2[0]),str(decimal2[1])
					self.parent.mapWidget.Signal_Nav_Route.emit(decimal[0],decimal[1],decimal2[0],decimal2[1])
					#print pointParser[0],':',pointParser[1]
				except:
					pass
			self.parent.mapWidget.repaint()

		def refreshNavRoutes(self):
			self.pointCoords.clear()
			routes = self.parent.mapWidget._route_points
			textString = ''
			for i in range(len(routes)):
				textString += ':'+str(routes[i][0][0])+','+str(routes[i][0][1])+'||'+':'+str(routes[i][1][0])+','+str(routes[i][1][1])+'\n'
				#print routes[i][0],routes[i][1],routes[i][2]
			self.pointCoords.setText(textString)
			
		def button_updateNavRoutes(self):
			if self.parent.accessType >= 1:
				self.updateNavRoutes()
			pass

		def button_refreshNavRoutes(self):
			if self.parent.accessType >= 1:
				self.refreshNavRoutes()
				if self.parent.accessType == 2:
					# navrouteStr = ''
					# navrouteList = []
					routes = self.parent.mapWidget._route_points
					# for i in range(len(routes)):
					# 	#navrouteList.append(Navroute(Navpt(routes[i][0][0],routes[i][0][1],''),Navpt(routes[i][1][0],routes[i][1][1],'')))
					# 	navrouteStr += str(routes[i][0][0])+","+str(routes[i][0][1])+","+str(routes[i][1][0])+","+str(routes[i][1][1])
					msg = self.parent.protocol.encodeNavRouteClearMsg()
					self.parent.sendMsg(msg)
					for i in range(len(routes)):
						msg = self.parent.protocol.encodeNavRouteMsg(routes[i])
						self.parent.sendMsg(msg)
					#self.parent.ROSThread.pub_BASE_Navroute_refresh.publish(navrouteList)
					print 'navroute refresh'
			pass

		def button_pushNavRoutes(self):
			routes = self.parent.mapWidget._route_points
			pointArr = []
			# print routes# [[[long,lati],[long,lati]],...]
			self.parent.navcomThread.endRoutes()
			for i in range(len(routes)):#go thru routes; order is B<-A
				if i == 0:
					pointArr.append(routes[i][1])
				pointArr.append(routes[i][0])
			self.parent.navcomThread.sendRoutes(pointArr)
			self.pushRoutes.setText('push'+str(len(pointArr))+":"+str(0))
			self.execRoutes.setEnabled(False)
			#1. Read in Navigation Routes and parse to point stack
			# routes = self.parent.mapWidget._route_points
			# pointArr = []
			# # print routes# [[[long,lati],[long,lati]],...]
			# for i in range(len(routes)):#go thru routes; order is B<-A
			# 	if i == 0:
			# 		pointArr.append(routes[i][1])
			# 	pointArr.append(routes[i][0])
			# msg = self.parent.rover.encode(RoverModel.NS_GPS, pointArr[0]+','+pointArr[1]+','+pointArr[2])
			# # SEND ME
			# self.parent.client._Server_UniCast_SendFunction(msg, SERVERCOMMS_UDP_IP, SERVERCOMMS_UDP_PORT)
			# print pointArr
			# pass

		def button_execNavRoutes(self):
			if self.parent.isExecRoute:
				self.parent.isExecRoute = False
				msg = self.parent.rover.encode(NS_EXEC, '1')
				self.parent.client._Server_UniCast_SendFunction(msg, SERVERCOMMS_UDP_IP, SERVERCOMMS_UDP_PORT)

		def button_clearNavRoutes(self):
			msg = self.parent.rover.encode(NS_CLEAR, '1')
			self.parent.client._Server_UniCast_SendFunction(msg, SERVERCOMMS_UDP_IP, SERVERCOMMS_UDP_PORT)
			self.parent.navcomThread.endRoutes()

	class EditTaskListWidget(QWidget):
		Signal_refreshTaskList = pyqtSignal()
		Signal_DoneTask = pyqtSignal()
		Signal_UndoTask = pyqtSignal()
		def __init__(self, width,height, grandparent, parent=None):
			super(NavigatorGUI.EditTaskListWidget, self).__init__(parent)
			self.parent = grandparent
			self.width = width #280
			self.height = height #250
			self.resize(self.width, self.height)
			self.routeLabel = QLabel('Tag:task_description',self)
			self.pointCoords = QTextEdit(self)
			self.pointCoords.resize(self.width,self.height-50)
			self.insertTasks = QPushButton("insert Tasks",self)
			self.refreshTasks = QPushButton("refresh Tasks",self)
			self.prevTask = QPushButton('prev',self)
			self.nextTask = QPushButton('next',self)
			self.prevTask.resize(40,30)
			self.nextTask.resize(40,30)
			self.pointCoords.setLineWrapMode(QTextEdit.NoWrap)
			self.insertTasks.clicked.connect(self.button_insertTaskList)
			self.refreshTasks.clicked.connect(self.button_refreshTaskList)
			self.nextTask.clicked.connect(self.button_DoneTask)
			self.prevTask.clicked.connect(self.button_PrevTask)
			self.Signal_refreshTaskList.connect(self.refreshTaskList)
			self.Signal_DoneTask.connect(self.Slot_DoneTask)
			self.Signal_UndoTask.connect(self.Slot_UndoTask)

			self.insertTasks.move(0,0)
			self.refreshTasks.move(self.insertTasks.width(),0)
			self.prevTask.move(self.insertTasks.width()+self.refreshTasks.width(),0)
			self.nextTask.move(self.insertTasks.width()+self.refreshTasks.width()+self.nextTask.width(),0)
			self.routeLabel.move(0,30)
			self.pointCoords.move(0,50)

		def insertTaskList(self):
			self.parent.taskWidget.Signal_Clear_Tasks.emit()
			lines = self.pointCoords.toPlainText().split('\n')
			for i in range(len(lines)):
				try:
					parser = lines[i].split(':')
					self.parent.taskWidget.Signal_Add_Task.emit(parser[0],parser[1],-1,False)
				except:
					pass
			self.parent.taskWidget.repaint()

		def refreshTaskList(self):
			self.pointCoords.clear()
			indexes = self.parent.taskWidget._index_list
			tasks = self.parent.taskWidget._task_list
			textString = ''
			for i in range(len(indexes)):
				textString += indexes[i]+':'+tasks[i]+'\n'
			self.pointCoords.setText(textString)

		def Slot_UndoTask(self):
			self.parent.taskWidget.Signal_Task_Not_Done.emit(self.parent.taskWidget._current_task)
			self.parent.taskWidget.repaint()

		def Slot_DoneTask(self):
			self.parent.taskWidget.Signal_Task_Done.emit(self.parent.taskWidget._current_task)
			self.parent.taskWidget.repaint()
			
		def button_insertTaskList(self):
			if self.parent.accessType >= 1:
				self.insertTaskList()
			pass
		def button_refreshTaskList(self):
			if self.parent.accessType >= 1:
				self.refreshTaskList()
				if self.parent.accessType == 2:
					taskListStr = ''
					#taskList = []
					for i in range(len(self.parent.taskWidget._index_list)):
						#taskList.append(Task(str(self.parent.taskWidget._index_list[i]),str(self.parent.taskWidget._task_list[i]),self.parent.taskWidget._time_list[i],self.parent.taskWidget._is_done[i]))
						taskListStr += str(self.parent.taskWidget._index_list[i])+","+str(self.parent.taskWidget._task_list[i])+","+str(self.parent.taskWidget._time_list[i])+","+str(self.parent.taskWidget._is_done[i])
						if i < len(self.parent.taskWidget._index_list)-1:
							taskListStr += '_'
					if not taskListStr == '':
						msg = self.parent.protocol.encodeTaskListMsg(self.parent.protocol.TASK_LIST_TYPE_REFRESH, taskListStr)
						self.parent.sendMsg(msg)
					#self.parent.ROSThread.pub_BASE_Task_refresh.publish(taskList)
			pass

		def button_DoneTask(self):
			if self.parent.accessType >= 1:
				self.Slot_DoneTask()
				if self.parent.accessType == 2:
					msg = self.parent.protocol.encodeTaskListMsg(self.parent.protocol.TASK_LIST_TYPE_DONE, '')
					self.parent.sendMsg(msg)
					#self.parent.ROSThread.pub_BASE_Task_mirror.publish(Task('','',-1,True))
		def button_PrevTask(self):
			if self.parent.accessType >= 1:
				self.Slot_UndoTask()
				if self.parent.accessType == 2:
					msg = self.parent.protocol.encodeTaskListMsg(self.parent.protocol.TASK_LIST_TYPE_UNDO, '')
					self.parent.sendMsg(msg)
					#self.parent.ROSThread.pub_BASE_Task_mirror.publish(Task('','',-1,False))

	class TimerControlWidget(QWidget):
		Signal_Update_Time = pyqtSignal(int)
		def __init__(self, width,height, grandparent, parent=None):
			super(NavigatorGUI.TimerControlWidget, self).__init__(parent)
			self.parent = grandparent
			self.width = width #280
			self.height = height #30
			self.resize(self.width, self.height)
			self.timerDisplay = QLineEdit('3600',self)
			self.controlButton = QPushButton('start',self)
			self.controlButton.clicked.connect(self.button_controlTimer)
			self.time = 3600
			self.isRunning = False

			self.timerDisplay.move(0,0)
			self.controlButton.move(self.timerDisplay.width(),0)
			self.timerThread = self.TimerThread(self)
			self.timerThread.start()

		def button_controlTimer(self):
			if self.parent.accessType >= 1:
				if self.isRunning:
					self.timerDisplay.setText(str(self.time))
					self.timerDisplay.setReadOnly(False)
					self.controlButton.setText('start')
					self.isRunning = False
					self.timerThread.Signal_Flip_Timer_Pause.emit()
				else:
					self.time = int(self.timerDisplay.text())
					self.timerDisplay.setText(self.timeToString(self.time))
					self.timerDisplay.setReadOnly(True)
					self.controlButton.setText('stop')
					self.isRunning = True
					self.timerThread.Signal_Flip_Timer_Pause.emit()

		def timeToString(self, times):
			minutes = int(times/60)
			seconds = times - minutes*60
			return str(minutes)+':'+str(seconds)

		def setTime(self):
			self.timerDisplay.setText(self.timeToString(self.time))

		class TimerThread(QThread):
			# Signals
			Signal_Flip_Timer_Pause = pyqtSignal()
			def __init__(self, parent=None):
				super(NavigatorGUI.TimerControlWidget.TimerThread, self).__init__(parent)
				self.parent = parent
				self.isThreadRunning = True
				self.isStopped = True
				self.Signal_Flip_Timer_Pause.connect(self.Slot_Flip_Timer_Pause)
				print 'CommsThread started'
			def run(self):
				print 'CommsThread running'
				while self.isThreadRunning:
					while self.isStopped:
						time.sleep(0.01)
					if self.parent.time > 0:
						self.parent.time -= 1
					self.parent.Signal_Update_Time.emit(self.parent.time)
					time.sleep(1)
			def Slot_Flip_Timer_Pause(self):
				self.isStopped = not self.isStopped

	class DataControlWidget(QWidget):
		def __init__(self, width,height, grandparent, parent=None):
			super(NavigatorGUI.DataControlWidget, self).__init__(parent)
			self.parent = grandparent
			self.width = width #280
			self.height = height #30
			self.resize(self.width, self.height)
			self.typeCombo = QComboBox(self)
			self.typeCombo.addItem("Read in UDP")
			self.typeCombo.addItem("local")
			self.typeCombo.addItem("Write out UDP")
			self.typeCombo.move(0,0)
			self.typeCombo.activated.connect(self.Slot_setType)
			self.typeCombo.setCurrentIndex(self.parent.accessType)

		def Slot_setType(self, choice):
			if choice == 0: # read
				self.parent.accessType = 0
			elif choice == 1: # local
				self.parent.accessType = 1
			elif choice == 2: # write
				self.parent.accessType = 2
			self.parent.setFocus()
			print 'set Type:',self.parent.accessType

	def clientConnect(self, client, interface):
		# client.Signal_Drive_Volt_Recv.connect(interface.Slot_Drive_Volt_Recv)
		# client.Signal_Drive_Volt_Send.connect(interface.Slot_Drive_Volt_Send)
		# client.Signal_ICCE_TiltX.connect(interface.Slot_ICCE_TiltX)
		# client.Signal_ICCE_TiltY.connect(interface.Slot_ICCE_TiltY)
		client.Signal_ICCE_Compass.connect(interface.Slot_ICCE_Compass)
		client.Signal_ICCE_GPS.connect(interface.Slot_ICCE_GPS)

	class GUITelemetryInterface():
		def __init__(self, parent=None):
			self.parent = parent

		def Slot_ICCE_Compass(self, heading):
			# self.parent.compassWidget.Signal_Update_Heading.emit(heading)
			self.parent.mapWidget.Signal_Update_Rover_Heading.emit(heading)
		def Slot_ICCE_GPS(self, lat, lon, alt):
			if lat == -1:
				lat = self.parent.mapWidget._rover_longitude
			if lon == -1:
				lon = self.parent.mapWidget._rover_latitude
			# self.parent.HUDWidget.Signal_Update_Rover_GPS.emit(lat, lon)
			if self.parent.isFollowingRover:
				self.parent._mapMid[0] = lat
				self.parent._mapMid[1] = lon
				self.parent.mapWidget.Signal_Update_Rover_GPS.emit(lat, lon)
				self.parent.mapWidget.Signal_View_GPS.emit(lat, lon)
		# def Slot_ICCE_TiltX(self, tiltX):
		# 	self.parent.roverTiltWidget.Signal_Update_TiltX.emit(tiltX)
		# def Slot_ICCE_TiltY(self, tiltY):
		# 	self.parent.roverTiltWidget.Signal_Update_TiltY.emit(tiltY)
		# def Slot_Drive_Volt_Send(self, motorNum, percentVoltage):
		# 	self.parent.driveBarWidget.Signal_Update_SentSpeed.emit(motorNum, percentVoltage)
		# def Slot_Drive_Volt_Recv(self, motorNum, percentVoltage):
		# 	self.parent.driveBarWidget.Signal_Update_Speed.emit(motorNum, percentVoltage)
		def Slot_Camera_Pan(self, heading):
			# self.parent.roverTiltWidget.Signal_Rotate_Base.emit(heading)
			# self.parent.HUDWidget.Signal_Move_Camera_X.emit((heading+180)%360)
			self.parent.mapWidget.Signal_Update_Camera_Heading.emit(0, heading)
		def Slot_Camera_Tilt(self, tilt):
			# self.parent.roverTiltWidget.Signal_Rotate_Angle.emit(tilt)
			# self.parent.HUDWidget.Signal_Move_Camera_Y.emit(tilt)
			self.parent.mapWidget.Signal_Update_Camera_Angle.emit(0, tilt)
		# def Slot_Server_Timer(self, time):
		# 	self.parent.taskListWidget.Signal_Update_Time.emit(time)
		# def Slot_Server_Link_Quality(self, lq):
		# 	self.parent.signalWidget.Signal_Update_Link_Quality.emit(lq)
		# def Slot_Server_Signal_Strength(self, ss):
		# 	self.parent.signalWidget.Signal_Update_Signal_Strength.emit(ss)
		# def Slot_Server_Ping(self, ping):
		# 	self.parent.signalWidget.Signal_Update_Ping.emit(ping)
		# def Slot_Client_Navpt(self, x,y, tag):
		# 	if tag == 'CLEAR':
		# 		self.parent.mapWidget.Signal_Clear_Nav_Point.emit()
		# 		self.parent.HUDWidget.Signal_Clear_Nav_Point.emit()
		# 	else:
		# 		self.parent.mapWidget.Signal_Nav_Point.emit(x,y,tag)
		# 		self.parent.HUDWidget.Signal_Insert_Nav_Point.emit(x,y, tag)
		# def Slot_Client_Navrt(self, x,y, x2,y2):
		# 	if x == y == x2 == y2 == -1:
		# 		self.parent.mapWidget.Signal_Clear_Nav_Route.emit()
		# 	else:
		# 		self.parent.mapWidget.Signal_Nav_Route.emit(x,y, x2,y2)
		# def Slot_Client_Task(self, tag,msg,time,done):
		# 	if tag == msg == 'clear':
		# 		self.parent.taskListWidget.Signal_Clear_Tasks.emit()
		# 	elif tag == msg == 'undo':
		# 		self.parent.taskListWidget.Signal_Task_Not_Done.emit(0)
		# 	elif tag == msg == 'done':
		# 		self.parent.taskListWidget.Signal_Task_Done.emit(0)
		# 	else:
		# 		self.parent.taskListWidget.Signal_Add_Task.emit(tag,msg,time,not done)

	class ClientComms(QThread):
		Signal_ICCE_Compass = pyqtSignal(int)
		Signal_ICCE_GPS = pyqtSignal(float, float, float)
		# Signal_ICCE_TiltX = pyqtSignal(float)
		# Signal_ICCE_TiltY = pyqtSignal(float)
		# Signal_Drive_Volt_Send = pyqtSignal(int, float) # DONE
		# Signal_Drive_Volt_Recv = pyqtSignal(int, float) # DONE

		# Signal_Camera_Pan = pyqtSignal(float)
		# Signal_Camera_Tilt = pyqtSignal(float)
		# Signal_Server_Timer = pyqtSignal(int)# NOT USED
		# Signal_Server_Link_Quality = pyqtSignal(str)
		# Signal_Server_Signal_Strength = pyqtSignal(str)
		# Signal_Server_Ping = pyqtSignal(str)
		#----------------       1. INIT COMMANDS        ----------------#
		def __init__(self, parent=None, rover=None):
			super(NavigatorGUI.ClientComms, self).__init__(parent)
			self.parent = parent
			self.rover = rover
			# self._client_TelemetryComms = MultiCastComms(MCAST_GRP, MCAST_PORT, MCAST_RECV_PORT)
			self._client_CommandComms = UniCastComms(TELEMETRY2_SEND_UDP_IP, TELEMETRY2_SEND_UDP_PORT)
			self.runThreads = True
			# self.client_MultiCastThread = RecvThread(self, 1, self._Server_MultiCast_RecvFunction, self._Server_MultiCast_ProcessTelemetry, THREAD_DELAY)
			self.client_UniCastThread = RecvThread(self, 2, self._Server_UniCast_RecvFunction, self._Server_UnitCast_RecvCommandSent, THREAD_DELAY)
			
		def startThreads(self):
			# self.client_MultiCastThread.start()
			self.client_UniCastThread.start()
			
		def endThreads(self):
			self.runThreads = False
			# self._Server_MultiCast_SendFunction('end')
			self._Server_UniCast_SendFunction('end', TELEMETRY2_SEND_UDP_IP, TELEMETRY2_SEND_UDP_PORT)
			
		#----------------       2. CLIENT MULTICAST FUNCTIONS       ----------------#
		# def _Server_MultiCast_RecvFunction(self):
		# 	return self._client_TelemetryComms.recvFunction()

		# def _Server_MultiCast_SendFunction(self, msg):
		# 	self._client_TelemetryComms.sendFunction(msg)

		# def _Server_MultiCast_ProcessTelemetry(self, num, msg):
		# 	#print 'thread(multi)',num,' got:',msg
		# 	# Process telemetry msg
		# 	if self.rover.updateTelemetry(msg):
		# 		print 'gps:',self.rover.icce.gps.lati, self.rover.icce.gps.long
		# 		cmdType, path, value = self.rover.decode(msg)
		# 		# if path == NS_MOTORFL_VOLT:
		# 		# 	self.Signal_Drive_Volt_Recv.emit(0, float(value))
		# 		# elif path == NS_MOTORFR_VOLT:
		# 		# 	self.Signal_Drive_Volt_Recv.emit(1, float(value))
		# 		# elif path == NS_MOTORRL_VOLT:
		# 		# 	self.Signal_Drive_Volt_Recv.emit(2, float(value))
		# 		# elif path == NS_MOTORRR_VOLT:
		# 		# 	self.Signal_Drive_Volt_Recv.emit(3, float(value))
		# 		# elif path == NS_IMU_X:
		# 		# 	self.Signal_ICCE_TiltX.emit(float(value))
		# 		# elif path == NS_IMU_Y:
		# 		# 	self.Signal_ICCE_TiltY.emit(float(value))
		# 		if path == NS_COMPASS_HEADING:
		# 			self.Signal_ICCE_Compass.emit(int(value))
		# 		elif path == NS_GPS_LATI:
		# 			self.Signal_ICCE_GPS.emit(float(value), -1)
		# 		elif path == NS_GPS_LONG:
		# 			self.Signal_ICCE_GPS.emit(-1, float(value))

		# def _Server_MultiCast_TestFunction(self, num, msg):
		# 	print 'thread(multi)',num,' got:',msg

		#----------------       3. CLIENT UNICAST FUNCTIONS     ----------------#
		def _Server_UniCast_SendFunction(self, msg, ip, port):
			self._client_CommandComms.sendFunction(msg, ip, port)

		def _Server_UniCast_RecvFunction(self):
			data,addr = self._client_CommandComms.recvFunction()
			return data

		def _Server_UnitCast_RecvCommandSent(self, num, msg):
			print 'thread(multi)',num,' got:',msg
			path,val,chksum = self.rover.decode(msg)
			value = val.split(',')
			if msg == 'end':
				# print 'sentEnd'
				self.runThreads = False
				self._client_TelemetryComms.close()
				self._client_CommandComms.close()
			elif path[0] == str(N_NAVCOM):
				if path[1] == str(N_ADD):
					navcom = self.parent.navcomThread
					pointNum = navcom.pointNum
					print 'n_add',value,pointNum,len(navcom.routes)
					if int(value[2]) < len(navcom.routes):
						route = navcom.routes[pointNum]
						print 'lat',route[1], value[0],str(route[1]) == str(value[0])
						print 'lon',route[0], value[1],str(route[0]) == str(value[1])
						print 'pt ',pointNum, value[2],int(pointNum) == int(value[2])
						if str(route[1]) == str(value[0]) and str(route[0]) == str(value[1]) and int(pointNum) == int(value[2]):
							print 'match!'
							if pointNum < len(navcom.routes):
								print 'increment!'
								self.parent.navcomThread.pointNum += 1
								self.parent.navRouteListWidget.pushRoutes.setText('push'+str(len(navcom.routes))+":"+str(pointNum))
				if path[1] == str(N_EXEC):
					self.parent.navRouteListWidget.execRoutes.setEnabled(False)
			elif self.rover.updateTelemetry(msg):
				path, value, chksum = self.rover.decode(msg)
				if path == NS_COMPASS_HEADING:
					self.Signal_ICCE_Compass.emit(int(value))
				elif path == NS_GPS:
					spi = value.split(',')
					self.Signal_ICCE_GPS.emit(float(spi[1]), float(spi[0]), float(spi[2]))
					print 'gps:',spi
				pass

		def _Server_UniCast_TestFunction(self, num, msg):
			print 'thread( uni )',num,' got:',msg

	def NAVCOM_ProcessFunct(self,num, routes, doPush, pointNum):
		# SEND POINTS
		if doPush:
			msg = self.rover.encode(NS_ADD, str(routes[pointNum][1])+','+str(routes[pointNum][0])+','+str(self.navcomThread.pointNum))
			# SEND ME
			self.client._Server_UniCast_SendFunction(msg, SERVERCOMMS_UDP_IP, SERVERCOMMS_UDP_PORT)
			
	def NAVCOM_CloseFunct(self):
		pass

	class NAVCOMThread(threading.Thread):
		def __init__(self, parent, num, threadFunc, closeFunc, delay):
			threading.Thread.__init__(self)
			self.parent = parent
			self.num = num
			self.threadFunc = threadFunc
			self.closeFunc = closeFunc
			self.delay = delay
			self.doPush = False
			self.pointNum = 0
			self.routes = []

		def run(self):
			print 'thread',self.num,':','started'
			isRunning = True
			while self.parent.runThreads and isRunning:
				if self.pointNum < len(self.routes):
					self.threadFunc(self.num, self.routes, self.doPush, self.pointNum)
				# self.pointNum += 1
				if self.pointNum == len(self.routes):
					self.endRoutes()
				time.sleep(self.delay)
				# break
			print 'thread',self.num,':','ended'

		def sendRoutes(self, routes):
			# start push
			self.routes = routes
			self.doPush = True
			self.pointNum = 0
			pass

		def endRoutes(self):
			# end push
			self.routes = []
			self.doPush = False
			self.pointNum = 0		
			# make exec possible
			self.parent.isExecRoute = True
			self.parent.navRouteListWidget.pushRoutes.setText('pushDone')
			self.parent.navRouteListWidget.execRoutes.setEnabled(True)

def main():
	app = QApplication(sys.argv)	# application object for PyQT
	gui = NavigatorGUI(1280,720)
	gui.show()
	# sys.exit(app.exec_())
	try:
		app.exec_()
	except KeyboardInterrupt:
		gui.runThreads = False
		gui.client.endThreads()
		gui.navcomThread.isRunning = False
		gui.sock.close()
		# event.accept()
		sys.exit()

if __name__ == '__main__':
	main()