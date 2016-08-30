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
from OperatorGUIWidgets import *

from datetime import datetime

import rospkg
#ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
path = r.get_path('mrp1_operator')

import rospy
from sensor_msgs.msg import Image
PKG = 'mrp1_operator' # this package name
import roslib; roslib.load_manifest(PKG)

from std_msgs.msg import String
from mrp1_operator.msg import GPSpt,Conveyor,Drive,Navpt,Navpt_list,Navroute,Navroute_list,Task,Task_list,Timer
from sensor_msgs.msg import NavSatFix

class ROSControlThread(QThread):
	Signal_ALES_Conveyor = pyqtSignal(float,float,float)
	Signal_ALES_Drive = pyqtSignal(float,float)
	Signal_ALES_GPS = pyqtSignal(float,float)
	Signal_ALES_Compass = pyqtSignal(float)
	Signal_Base_Conveyor = pyqtSignal(float,float,float)
	Signal_Base_Drive = pyqtSignal(float,float)

	Signal_Base_Timer = pyqtSignal(int)
	Signal_Base_Navpt_mirror = pyqtSignal(float,float,str)
	Signal_Base_Navpt_refresh = pyqtSignal(Navpt_list)
	Signal_Base_Navroute_mirror = pyqtSignal(float,float,str,float,float,str)
	Signal_Base_Navroute_refresh = pyqtSignal(Navroute_list)
	Signal_Base_Task_mirror = pyqtSignal(bool)
	Signal_Base_Task_refresh = pyqtSignal(Task_list)
	def __init__(self, parent=None):
		super(ROSControlThread, self).__init__(parent)
		self.parent = parent
		self.isRunning = True
		rospy.init_node('Navigator_node', anonymous=True)#this node's name = listener
		self.sub_ALES_GPS = rospy.Subscriber("fix", NavSatFix, self.get_ALES_GPS)

		self.sub_BASE_Timer = rospy.Subscriber("BASE_Timer", Timer, self.get_BASE_Timer)
		self.sub_BASE_Navpt_mirror = rospy.Subscriber("BASE_Navpt_mirror", Navpt, self.get_BASE_Navpt_mirror)
		self.sub_BASE_Navpt_refresh = rospy.Subscriber("BASE_Navpt_refresh", Navpt_list, self.get_BASE_Navpt_refresh)
		self.sub_BASE_Navroute_mirror = rospy.Subscriber("BASE_Navroute_mirror", Navroute, self.get_BASE_Navroute_mirror)
		self.sub_BASE_Navroute_refresh = rospy.Subscriber("BASE_Navroute_refresh", Navroute_list, self.get_BASE_Navroute_refresh)
		self.sub_BASE_Task_mirror = rospy.Subscriber("BASE_Task_mirror", Task, self.get_BASE_Task_mirror)
		self.sub_BASE_Task_refresh = rospy.Subscriber("BASE_Task_refresh", Task_list, self.get_BASE_Task_refresh)

		self.pub_BASE_Timer = rospy.Publisher("BASE_Timer", Timer)
		self.pub_BASE_Navpt_mirror = rospy.Publisher("BASE_Navpt_mirror", Navpt)
		self.pub_BASE_Navpt_refresh = rospy.Publisher("BASE_Navpt_refresh", Navpt_list)
		self.pub_BASE_Navroute_mirror = rospy.Publisher("BASE_Navroute_mirror", Navroute)
		self.pub_BASE_Navroute_refresh = rospy.Publisher("BASE_Navroute_refresh", Navroute_list)
		self.pub_BASE_Task_mirror = rospy.Publisher("BASE_Task_mirror", Task)
		self.pub_BASE_Task_refresh = rospy.Publisher("BASE_Task_refresh", Task_list)

	def get_ALES_GPS(self, gpsMsg):
		self.Signal_ALES_GPS.emit(gpsMsg.longitude,gpsMsg.latitude)

	def get_BASE_Timer(self, time):
		#do nothing
		if self.parent.accessType == 0:
			print 'got Time!',time.time
			self.parent.Signal_Update_Time.emit(time.time)
		pass
	def get_BASE_Navpt_mirror(self, Navpt_msg):
		print 'navpt mirror'#, Navpt_msg
		#do nothing
		if self.parent.accessType == 0:
			self.parent.mapWidget.Signal_Nav_Point.emit(Navpt_msg.longitude,Navpt_msg.latitude,Navpt_msg.tag)
			self.parent.mapWidget.Signal_Repaint.emit()
			self.parent.navPointListWidget.Signal_refreshList.emit()
 		pass
	def get_BASE_Navpt_refresh(self, Navpt_list_msg):
		print 'navpt refresh'#, Navpt_list_msg
		if self.parent.accessType == 0:
			self.parent.mapWidget.Signal_Clear_Nav_Point.emit()
			for i in range(len(Navpt_list_msg.list)):
				longitude = Navpt_list_msg.list[i].longitude
				latitude = Navpt_list_msg.list[i].latitude
				tag = Navpt_list_msg.list[i].tag
				self.parent.mapWidget.Signal_Nav_Point.emit(longitude,latitude,tag)
			self.parent.mapWidget.Signal_Repaint.emit()
			self.parent.navPointListWidget.Signal_refreshList.emit()
		pass
	def get_BASE_Navroute_mirror(self, Navroute_msg):
		print 'navroute mirror'
		#do nothing
		if self.parent.accessType == 0:
			self.parent.mapWidget.Signal_Nav_Route.emit(Navroute_msg.pt1.longitude,Navroute_msg.pt1.latitude,Navroute_msg.pt2.longitude,Navroute_msg.pt2.latitude)
			self.parent.mapWidget.Signal_Repaint.emit()
			self.parent.navRouteListWidget.Signal_refreshList.emit()
		pass
	def get_BASE_Navroute_refresh(self, Navroute_list_msg):
		print 'navroute refresh'
		if self.parent.accessType == 0:
			self.parent.mapWidget.Signal_Clear_Nav_Route.emit()
			for i in range(len(Navroute_list_msg.list)):
				longitude = Navroute_list_msg.list[i].pt1.longitude
				latitude = Navroute_list_msg.list[i].pt1.latitude
				longitude2 = Navroute_list_msg.list[i].pt2.longitude
				latitude2 = Navroute_list_msg.list[i].pt2.latitude
				self.parent.mapWidget.Signal_Nav_Route.emit(longitude,latitude, longitude2,latitude2)
			self.parent.mapWidget.Signal_Repaint.emit()
			self.parent.navRouteListWidget.Signal_refreshList.emit()
			pass
	def get_BASE_Task_mirror(self, Task_msg):
		print 'got task:'#,Task_msg
		#do nothing
		if self.parent.accessType == 0:
			isDone = Task_msg.isDone
			if isDone:
				self.parent.taskListWidget.Signal_DoneTask.emit()
			else:
				self.parent.taskListWidget.Signal_UndoTask.emit()
			#self.parent.taskWidget.Signal_Add_Task.emit(Task_msg.tag,Task_msg.task,Task_msg.time,Task_msg.isDone)
			#self.parent.taskWidget.Signal_Repaint.emit()
			pass
		pass
	def get_BASE_Task_refresh(self, Task_list_msg):
		print 'got task list:'#,Task_list_msg
		if self.parent.accessType == 0:
			self.parent.taskWidget.Signal_Clear_Tasks.emit()
			for i in range(len(Task_list_msg.list)):
				tag = Task_list_msg.list[i].tag
				task = Task_list_msg.list[i].task
				time = Task_list_msg.list[i].time
				isDone = Task_list_msg.list[i].isDone
				self.parent.taskWidget.Signal_Add_Task.emit(tag,task,time,isDone)
			self.parent.taskWidget.Signal_Repaint.emit()
			self.parent.taskListWidget.Signal_refreshTaskList.emit()
			pass
		pass

class NavigatorGUI(QMainWindow):
	Signal_Update_Time = pyqtSignal(int)
	def __init__(self, width,height):
		super(NavigatorGUI, self).__init__()
		self.setWindowTitle('NavigatorGUI')
		self.setWindowIcon(QIcon('hp_cat.png'))
		self.width = 1280
		self.height = 760
		self.resize(self.width,self.height)
		self.Signal_Update_Time.connect(self._Slot_Update_Time)

		self.startMoveX = 0
		self.startMoveY = 0
		self.endMoveX = 0
		self.endMoveY = 0
		self.isLeftButton = False
		self.inputMode = 0#0=point 1=route
		self.prevRouteEnd = [0.0,0.0]
		self.map_type = 0#0=topo 1=aerial
		self.accessType = 1 # 0:read,1:offline,2:write
		self.ROSThread = ROSControlThread(self)
		self.ROSThread.Signal_ALES_GPS.connect(self._Slot_ALES_GPS)

		self.rPixel = [53,30]

		self.initUI()
		self.setFocus()

	def initUI(self):
		self.splitter = QSplitter(Qt.Horizontal, self)
		self.splitter.resize(self.width,self.height)
		#self.splitter.setFrameShape(QFrame.StyledPanel)
		self.boxWidget = QWidget(self)
		self.mapBoxWidget = QWidget(self)

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
		self.GPSDpWidget.Signal_Update_GPS.emit(self._mapMid[0],self._mapMid[1])
		
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
		self.inputLabel = QLabel("Point ", self.boxWidget)#space is to make default size show "Route" properly
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
			self.ROSThread.pub_BASE_Timer.publish(time)

	def _Slot_ALES_GPS(self, longitude, latitude):
		self.mapWidget.Signal_Update_Rover_GPS.emit(longitude,latitude)
		self.GPSDpWidget.Signal_Update_GPS.emit(longitude,latitude)

	def keyPressEvent(self, e):
		if e.key() == Qt.Key_Escape: # Esc
			self.close()
		elif e.key() == Qt.Key_Slash:
			pass
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
			if self.inputMode == 1:
				self.inputMode = 0
				self.prevRouteEnd = [0.0,0.0]
				self.inputLabel.setText("Point")
			else:
				self.inputMode = 1
				self.inputLabel.setText("Route")
			print 'inputMode:',self.inputMode
		elif e.key() == Qt.Key_M:
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
					self.ROSThread.pub_BASE_Navpt_mirror.publish(Navpt(navX,navY,''))
				#print 'NavPt:',navX,' ',navY
			elif self.inputMode == 1 and self.accessType >= 1:
				if self.prevRouteEnd[0] == 0.0 and self.prevRouteEnd[1] == 0.0:
					self.prevRouteEnd = [navX,navY]
				else:
					self.mapWidget.Signal_Nav_Route.emit(navX,navY, self.prevRouteEnd[0],self.prevRouteEnd[1])
					if self.accessType == 2:#send route added to other clients; mirror
						self.ROSThread.pub_BASE_Navroute_mirror.publish(Navroute(Navpt(navX,navY,''),Navpt(self.prevRouteEnd[0],self.prevRouteEnd[1],'')))
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
					navptList = []
					points = self.parent.mapWidget._nav_points
					for i in range(len(points)):
						navptList.append(Navpt(points[i][0],points[i][1],str(points[i][2])))
					self.parent.ROSThread.pub_BASE_Navpt_refresh.publish(navptList)
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
			self.insertRoutes = QPushButton("insert Routes",self)
			self.refreshRoutes = QPushButton("Refresh Routes",self)
			self.pointCoords.setLineWrapMode(QTextEdit.NoWrap)
			self.insertRoutes.clicked.connect(self.button_updateNavRoutes)
			self.refreshRoutes.clicked.connect(self.button_refreshNavRoutes)
			self.Signal_refreshList.connect(self.refreshNavRoutes)

			self.insertRoutes.move(0,0)
			self.refreshRoutes.move(self.insertRoutes.width(),0)
			self.routeLabel.move(self.insertRoutes.width()+self.refreshRoutes.width(),5)
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
					navrouteList = []
					routes = self.parent.mapWidget._route_points
					for i in range(len(routes)):
						navrouteList.append(Navroute(Navpt(routes[i][0][0],routes[i][0][1],''),Navpt(routes[i][1][0],routes[i][1][1],'')))
					self.parent.ROSThread.pub_BASE_Navroute_refresh.publish(navrouteList)
					print 'navroute refresh'
			pass

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
					taskList = []
					for i in range(len(self.parent.taskWidget._index_list)):
						taskList.append(Task(str(self.parent.taskWidget._index_list[i]),str(self.parent.taskWidget._task_list[i]),self.parent.taskWidget._time_list[i],self.parent.taskWidget._is_done[i]))
					self.parent.ROSThread.pub_BASE_Task_refresh.publish(taskList)
			pass

		def button_DoneTask(self):
			if self.parent.accessType >= 1:
				self.Slot_DoneTask()
				if self.parent.accessType == 2:
					self.parent.ROSThread.pub_BASE_Task_mirror.publish(Task('','',-1,True))
		def button_PrevTask(self):
			if self.parent.accessType >= 1:
				self.Slot_UndoTask()
				if self.parent.accessType == 2:
					self.parent.ROSThread.pub_BASE_Task_mirror.publish(Task('','',-1,False))

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
			self.typeCombo.addItem("ROS-Read")
			self.typeCombo.addItem("local")
			self.typeCombo.addItem("ROS-Write")
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

# def main():
# 	app = QApplication(sys.argv)	# application object for PyQT
# 	gui = NavigatorGUI(1000,700)
# 	gui.show()
# 	sys.exit(app.exec_())

# if __name__ == '__main__':
# 	main()