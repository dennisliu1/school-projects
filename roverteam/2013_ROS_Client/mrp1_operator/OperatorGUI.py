#!/usr/bin/python
# -*- coding: utf-8 -*-
# Udemy, TradingPAY.com
# mark@bnotions.com - contact for info and questions for gurus
# 
'''
1*_10_* = Voltage
  _11_* = Current
  _12_* = Velocity
50_1_*  = IMU X Vector
50_2_*  = IMU Y Vector
50_3_*  = IMU Z Vector
50_4_*  = IMU Yaw
50_5_*  = IMU Pitch
50_6_*  = IMU Roll
51_1_*  = Compass Heading(degree)
52_1_*  = GPS Reading
53_1_*  = Rover Tilt X
53_2_*  = Rover Tilt Y
70_1_*  = OBC Signal Strength Reading
90_1    = Operator GUI Quit Signal
90_2_*  = Timer time(seconds)
'''
import gobject

import sys
import socket
import time
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from datetime import datetime

# import pygst
# pygst.require("0.10")
# import gst
# import gobject

# import struct
# hdr_fmt = "!LQQi"
# hdr_len = struct.calcsize(hdr_fmt)

from OperatorGUIWidgets import *

import rospkg
#ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
path = r.get_path('mrp1_operator')

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
PKG = 'mrp1_operator' # this package name
import roslib; roslib.load_manifest(PKG)

from std_msgs.msg import String
from mrp1_operator.msg import GPSpt,Conveyor,Drive,Navpt,Navpt_list,Navroute,Navroute_list,Task,Task_list,Timer,Compass,Ping,Signals,OperatorGUI_Status,Astrobox,PTZ
from sensor_msgs.msg import NavSatFix
#import GPSpt
'''
- goto next camera (main+pip)
- play/pause/stop camera (main+pip)
- Clear HUD
- take screenshot
- expand minimap
- zoom map
- task list size
- scroll task list(?)
- show help
- switch map type
'''
class ROSThread(QThread):
	'''
	self.sub_OperatorGUI_prevCamera
	self.sub_OperatorGUI_nextCamera
	self.sub_OperatorGUI_toggleCamera
	self.sub_OperatorGUI_prevPIP
	self.sub_OperatorGUI_nextPIP
	self.sub_OperatorGUI_togglePIP
	self.sub_OperatorGUI_toggleHUD
	self.sub_OperatorGUI_expandMinimap
	self.sub_OperatorGUI_switchMap
	self.sub_OperatorGUI_toggleHelp
	'''

	Signal_ALES_GPS = pyqtSignal(float,float)
	Signal_ALES_Compass = pyqtSignal(float)
	Signal_ALES_Signals = pyqtSignal(str,str)
	Signal_BASE_Ping = pyqtSignal(str)

	Signal_ALES_PTZ = pyqtSignal(float,float)
	Signal_ALES_Conveyor = pyqtSignal(float,float,float)
	Signal_ALES_Drive = pyqtSignal(float,float)
	Signal_Base_Conveyor = pyqtSignal(float,float,float)
	Signal_Base_Drive = pyqtSignal(float,float)
	Signal_Base_Timer = pyqtSignal(int)
	Signal_Base_Navpt_mirror = pyqtSignal(float,float,str)
	Signal_Base_Navpt_refresh = pyqtSignal(Navpt_list)
	Signal_Base_Navroute_mirror = pyqtSignal(float,float,str,float,float,str)
	Signal_Base_Navroute_refresh = pyqtSignal(Navroute_list)
	Signal_Base_Task_mirror = pyqtSignal(bool)
	Signal_Base_Task_refresh = pyqtSignal(Task_list)
	Signal_OperatorGUI_Status = pyqtSignal(OperatorGUI_Status)
	Signal_ALES_AstronautBox_Trigger = pyqtSignal(float,float,float,float,float)
	def __init__(self, parent=None):
		super(ROSThread, self).__init__(parent)
		self.parent = parent
		self.isRunning = True
		rospy.init_node('GUI_listener', anonymous=True)#this node's name = listener
		self.sub_ALES_Conveyor = rospy.Subscriber("ALES_send_conveyor", Conveyor, self.get_ALES_conveyor)
		self.sub_ALES_Drive = rospy.Subscriber("ALES_send_drive", Twist, self.get_ALES_drive)
		self.sub_ALES_GPS = rospy.Subscriber("fix", NavSatFix, self.get_ALES_GPS)
		self.sub_ALES_Compass = rospy.Subscriber("ALES_Send_Compass", Compass, self.get_ALES_Compass)
		self.sub_BASE_Ping = rospy.Subscriber("BASE_Ping", Ping, self.get_BASE_Ping)
		self.sub_ALES_Signals = rospy.Subscriber("ALES_Send_Signals", Signals, self.get_ALES_Signals)

		self.sub_ALES_PTZ = rospy.Subscriber("ALES_PTZ",PTZ, self.get_ALES_PTZ)
		self.sub_ALES_Astrobox = rospy.Subscriber("ALES_Astrobox",Astrobox,self.get_ALES_Astrobox)
		self.sub_BASE_Conveyor = rospy.Subscriber("BASE_conveyor", Conveyor, self.get_BASE_conveyor)
		self.sub_BASE_Drive = rospy.Subscriber("BASE_drive", Twist, self.get_BASE_drive)
		self.sub_BASE_Timer = rospy.Subscriber("BASE_Timer", Timer, self.get_BASE_timer)
		self.sub_BASE_Navpt_mirror = rospy.Subscriber("BASE_Navpt_mirror", Navpt, self.get_BASE_navpt_mirror)
		self.sub_BASE_Navpt_refresh = rospy.Subscriber("BASE_Navpt_refresh", Navpt_list, self.get_BASE_navpt_refresh)
		self.sub_BASE_Navroute_mirror = rospy.Subscriber("BASE_Navroute_mirror", Navroute, self.get_BASE_navroute_mirror)
		self.sub_BASE_Navroute_refresh = rospy.Subscriber("BASE_Navroute_refresh", Navroute_list, self.get_BASE_navroute_refresh)
		self.sub_BASE_Task_mirror = rospy.Subscriber("BASE_Task_mirror", Task, self.get_BASE_Task_mirror)
		self.sub_BASE_Task_refresh = rospy.Subscriber("BASE_Task_refresh", Task_list, self.get_BASE_Task_refresh)
		self.sub_OperatorGUI_Status = rospy.Subscriber("BASE_OperatorGUI_Status", OperatorGUI_Status, self.get_OperatorGUI_Status)

	def run(self):
		print 'start thread'
		# spin() simply keeps python from exiting until this node is stopped
		# while not rospy.is_shutdown():
		# 	time.sleep(1)
		#rospy.spin()
		pass

	def get_ALES_PTZ(self, ptzMSG):
		self.Signal_ALES_PTZ.emit(ptzMSG.heading,ptzMSG.angle)
		pass
	def get_ALES_Astrobox(self, astroboxMsg):
		print "got astrobox",astroboxMsg
		self.Signal_ALES_AstronautBox_Trigger.emit(astroboxMsg.speed1,astroboxMsg.speed2,astroboxMsg.speed3,astroboxMsg.speed4,astroboxMsg.speed5)

	def get_BASE_Ping(self, pingmsg):
		print 'got ping!', pingmsg.time
		self.Signal_BASE_Ping.emit(pingmsg.time)
		pass
	def get_ALES_Signals(self, signalmsg):
		self.Signal_ALES_Signals.emit(signalmsg.quality,signalmsg.level)

	def get_ALES_conveyor(self, conveyor):
		self.Signal_ALES_Conveyor.emit(conveyor.front, conveyor.back,conveyor.vertical)
		pass
	def get_ALES_drive(self, drive):
		self.Signal_ALES_Drive.emit(drive.linear.x,drive.angular.z)
		pass
	def get_ALES_GPS(self, gps):
		print 'got GPS',gps
		self.Signal_ALES_GPS.emit(gps.longitude,gps.latitude)
		pass
	def get_ALES_Compass(self, compassmsg):
		print 'got compass!',compassmsg.heading
		self.Signal_ALES_Compass.emit(compassmsg.heading)
		pass
	def get_BASE_conveyor(self, conveyor):
		self.Signal_Base_Conveyor.emit(conveyor.front_conveyor, conveyor.back_conveyor,conveyor.vertical)
		pass
	def get_BASE_drive(self, drive):
		self.Signal_Base_Drive.emit(drive.left,drive.right)
		pass
	def get_BASE_timer(self, timer):
		print 'timer:',timer
		self.Signal_Base_Timer.emit(timer.time)
		pass
	def get_BASE_navpt_mirror(self, navpt):
		self.Signal_Base_Navpt_mirror.emit(navpt.longitude,navpt.latitude,navpt.tag)
		pass
	def get_BASE_navpt_refresh(self, navpt_list):
		print 'navpt refresh'
		self.Signal_Base_Navpt_refresh.emit(navpt_list)
		pass
	def get_BASE_navroute_mirror(self, navroute):
		self.Signal_Base_Navroute_mirror.emit(navroute.pt1.longitude,navroute.pt1.latitude,navroute.pt1.tag, navroute.pt2.longitude,navroute.pt2.latitude,navroute.pt2.tag)
		pass
	def get_BASE_navroute_refresh(self, navroute_list):
		print 'navroute refresh'
		self.Signal_Base_Navroute_refresh.emit(navroute_list)
		pass
	def get_BASE_Task_mirror(self, taskmsg):
		#print 'Task:',taskmsg
		self.Signal_Base_Task_mirror.emit(taskmsg.isDone)
		pass
	def get_BASE_Task_refresh(self, taskList):
		#print 'Task List:',taskList
		self.Signal_Base_Task_refresh.emit(taskList)
		pass
	def get_OperatorGUI_Status(self, OperatorGUI_Statusmsg):
		#print 'got status:',OperatorGUI_Statusmsg
		self.Signal_OperatorGUI_Status.emit(OperatorGUI_Statusmsg)
		pass

class RoverModel():
	wheelSpeed = [0.0,0.0, 0.0,0.0]# FL,RL, FR,RR
	cameras = [[0.0,0.0,60.0],[90.0,0.0,60.0],[180.0,0.0,60.0],[270.0,0.0,60.0]]# front,right,back,left camera (heading,angle)
	GPS = [0.0,0.0] # latitude,longitude (wgs84, decimal)
	astronautBox = [False,False,False,False,False]# Whether box has been opened yet
class GUIModel():
	pass

##############                    MAIN GUI                        ##############
class OperatorGUI(QMainWindow):
	Signal_OperatorGUI_Quit = pyqtSignal() #signal to send for closing
	def __init__(self):
		super(OperatorGUI, self).__init__()
		ret = gst.element_register(VideoSink, 'VideoSink')
		gobject.threads_init()

		#self.setAttribute(Qt.WA_NoSystemBackground)
		#self.setAutoFillBackground(True)
		self.setWindowTitle('OperatorGUI')
		self.setWindowIcon(QIcon(path+'hp_cat.png'))
		self.setToolTip('This is a <b>QWidget</b> widget - GUI Base')
		#self.comms = CommsThread(self.Signal_OperatorGUI_Quit, self)
		self.ROSComms = ROSThread(self)

		self.showHUD = True
		self.roverModel = RoverModel()

		#self.pipeline = gst.parse_launch("autovideosrc ! videorate ! videoscale ! video/x-raw-rgb,width=480,height=360 ! jpegenc ! VideoSink name=\"videosink0\"")
		#self.pipeline = gst.parse_launch("autovideosrc ! videorate ! videoscale ! video/x-raw-rgb ! jpegenc ! VideoSink name=\"videosink0\" sync=false")
		#self.pipeline = gst.parse_launch("rtspsrc location=\"rtsp://marsrover-titan:8554/video0\" ! rtph264depay  ! h264parse ! ffdec_h264 ! queue ! jpegenc ! VideoSink name=\"videosink0\" sync=false")

		# self.pipeline = gst.Pipeline("player")
		# self.source = gst.element_factory_make("autovideosrc", "autovideosrc0")
		# self.videorate = gst.element_factory_make("videorate", "videorate0")
		# self.videoscale = gst.element_factory_make("videoscale", "videoscale0")

		#self.sink = self.pipeline.get_by_name("videosink0")
		# self.sink._index = 0
		#self.videoSet = [[self.pipeline,self.sink,"Video0"]]
		self.initUI()
		
	def initUI(self):
		# 1.variables
		QToolTip.setFont(QFont('SansSerif', 10))
		self.OperatorGUI_Window_Size = QSize(1200,675)
		self.ratios = [53.0,30.0]
		self.rPixel = [self.OperatorGUI_Window_Size.width()/self.ratios[0],self.OperatorGUI_Window_Size.height()/self.ratios[1]]
		# 1.1 Rover Telemetry
		self.rover_Telemetry_IMU = [0,0,0, 0,0,0] #tiltX/Y/Z, yaw/pitch/roll
		self.rover_Telemetry_Compass = 0 #compass heading
		self.rover_Telemetry_Voltages = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self.rover_Telemetry_Currents = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self.rover_Telemetry_Velocities = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self.rover_Telemetry_GPS_longitude  = -1#x; init as middle of map
		self.rover_Telemetry_GPS_latitude   = -1#y; init as middle of map
		self.rover_Telemetry_Signal_Strength = "5/9"
		self.rover_Telemetry_Camera_Heading = 0
		self.rover_Telemetry_Camera_Angle = 0
		self.rover_Telemetry_Rover_Heading = 0
		self.map_type = 0

		self.nav_points = []
		self.nav_routes = []

		# 1.2 Base Telemetry
		self.base_Telemetry_Timer = 3600
		self.msgIsText = True #messageWidget
		self.currentTask = 1 #task list widget
		self.showColumns = 2 #number of columns shown for task list widget
		self.currentMenuCamera = 0
		
		# self.showFullScreen()
		self.resize(self.OperatorGUI_Window_Size)
		center(self)

		# 4.populate Widgets
		self.NUMBER_CAMERAS = 3
		self.cameraFOV = [0.0,90.0]
		self.pipelines = GStreamerPipeline()
		self.pipelines.startup(self.NUMBER_CAMERAS)
		self.pipelines.Trigger_New_Frame().connect(self.Slot_Trigger_New_Frame)

		self.videoDisplayWidget = VideoDisplayWidget(self.OperatorGUI_Window_Size.width(), self.OperatorGUI_Window_Size.height(), self.pipelines, self)
		self.videoDisplayWidget.move(0,0)
		self.videoControlWidget_ratio = [4.0,1.5]
		width = self.rPixel[0]*self.videoControlWidget_ratio[0]
		height = self.rPixel[1]*self.videoControlWidget_ratio[1]
		self.videoControlWidget = CameraStatusWidget(width,height, self)
		self.videoControlWidget.move(self.OperatorGUI_Window_Size.width()-self.videoControlWidget.width,0)
		self.videoControlWidget.Signal_Camera_Set.emit("video0")#default

		# PIP
		self.videoDisplayWidget2_ratio = [8.0,6.0]
		width = self.rPixel[0]*self.videoDisplayWidget2_ratio[0]
		height = self.rPixel[1]*self.videoDisplayWidget2_ratio[1]
		self.videoDisplayWidget2 = VideoDisplayWidget(width,height,self.pipelines, self)
		self.videoDisplayWidget2.move(self.OperatorGUI_Window_Size.width()-self.videoDisplayWidget2.width,self.videoControlWidget.height)
		self.videoControlWidget2_ratio = [4.0,1.5]
		width = self.rPixel[0]*self.videoControlWidget2_ratio[0]
		height = self.rPixel[1]*self.videoControlWidget2_ratio[1]
		self.videoControlWidget2 = CameraStatusWidget(width,height, self)
		self.videoControlWidget2.move(self.OperatorGUI_Window_Size.width()-self.videoControlWidget2.width,self.videoControlWidget.height)
		self.videoControlWidget2.Signal_Camera_Set.emit("video0")#default

		# HUD Overlay Widget - DONE
		# width, height, fov, icon size                                                                    
		self.HUDOverlay = HUDOverlayWidget(self.OperatorGUI_Window_Size.width(),self.OperatorGUI_Window_Size.height(), 60, 9,self)
		self.HUDOverlay.move(0,0)

		# 4.2 Message Widget - DONE - SCALED
		# self.messageWidgetOffset = 30
		# self.msgWidget = MessageWidget(self.OperatorGUI_Window_Size.width()/4, self.OperatorGUI_Window_Size.width()/(4*4.5), self)
		# self.msgWidget.move(self.OperatorGUI_Window_Size.width()/2-self.msgWidget.width/2,self.messageWidgetOffset)

		# 4.4 rover parts info - DONE - SCALED
		# roverGraphicWidth = self.OperatorGUI_Window_Size.width()/10
		# roverGraphicHeight = self.OperatorGUI_Window_Size.height()/4
		# self.roverGraphicWidget = RoverGraphicWidget(roverGraphicWidth,roverGraphicHeight, self)
		# self.roverGraphicWidget.move(self.OperatorGUI_Window_Size.width()-self.roverGraphicWidget.width, self.OperatorGUI_Window_Size.height()/2-self.roverGraphicWidget.height/2)

		# 4.5 acceleration bar - DONE - SCALED
		self.driveBarWidget_ratio = [8.0,1.5]
		width = self.rPixel[0]*self.driveBarWidget_ratio[0]
		height = self.rPixel[1]*self.driveBarWidget_ratio[1]
		self.ruler_tick_size = width/20.0  # how far apart small ticks are
		self.ruler_tick_length = height/4.0# how long each small tick is
		self.large_tick_length = self.ruler_tick_length*2.0# how long each small tick is
		self.ruler_tick_num = 20  # number of small ticks
		self.large_tick_offset = 5# every X small tick is a large tick
		self.driveBarWidget = DriveBarWidget(width,height, self.ruler_tick_size, self.ruler_tick_length, self.large_tick_length, self.ruler_tick_num, self.large_tick_offset, self)
		self.driveBarWidget.move(self.OperatorGUI_Window_Size.width()-self.driveBarWidget.width, 
								 self.OperatorGUI_Window_Size.height()-self.driveBarWidget.height)

		# 4.6 minimap
		self.minimapWidget_ratio = [7.0,7.0]
		width = self.rPixel[0]*self.minimapWidget_ratio[0]
		height = self.rPixel[1]*self.minimapWidget_ratio[1]
		roverRadius = (width+height)/100.0
		self.isMinimapLarge = False
		self.minimapWidget = MinimapWidget(width,height, roverRadius, self)
		self.minimapWidget.move(0, self.OperatorGUI_Window_Size.height()-self.minimapWidget.height)

		# 4.7 compass - DONE - SCALED
		# Width, Height, space between each tick, font size
		self.compass_ratio = [7.0,1.0]
		self.compass_fov = 90.0
		width = self.rPixel[0]*self.compass_ratio[0]
		height = self.rPixel[1]*self.compass_ratio[1]
		self.compass_font = height/2
		self.compass = CompassWidget(width,height, width/self.compass_fov, self.compass_font, self)
		#self.compass.setAttribute(Qt.WA_OpaquePaintEvent, True)
		self.compass.move(0, self.OperatorGUI_Window_Size.height()-self.minimapWidget.height-self.compass.height)

		self.compass_ratio = [7.0,1.0]
		self.compass_fov = 90.0
		width = self.rPixel[0]*self.compass_ratio[0]
		height = self.rPixel[1]*self.compass_ratio[1]
		self.compass_font = height/2
		self.compassCamera = CompassWidget(width,height, width/self.compass_fov, self.compass_font, self)
		self.compassCamera.move(self.OperatorGUI_Window_Size.width()/2-self.compassCamera.width/2,self.OperatorGUI_Window_Size.height()-self.compassCamera.height)
		# 4.8 timer - DONE
		# self.timer_ratio = []
		# width = self.OperatorGUI_Window_Size.width() * (self.compass_ratio[0]/self.ratios[0])
		# height = self.OperatorGUI_Window_Size.height() * (self.compass_ratio[1]/self.ratios[1])
		# self.timer = TimerWidget(self.base_Telemetry_Timer, self)
		# self.timer.resize(150,50)
		# self.timer.move(self.minimapWidget.width, self.OperatorGUI_Window_Size.height()-self.timer.height())

		# 4.9 Task List - DONE
		# margin size, phase1 width, phase2 width, phase3 width, fontSize, # rows to show
		self.tasklistWidget_ratio = [6.5,4.0]
		width = self.rPixel[0]*self.tasklistWidget_ratio[0]
		height = self.rPixel[1]*self.tasklistWidget_ratio[1]
		margin = width/10.0
		phase1 = 3
		phase2 = 15
		phase3 = 5
		numRows= 10
		fontLen= (height/numRows)
		#print 'task list:',width,height,':',margin,phase1,phase2,phase3,fontLen,numRows
		self.tasklistWidget = TaskListWidget(margin,phase1,phase2,phase3,fontLen,numRows, self)
		self.tasklistWidget.move(0,0)
		self.tasklistWidget.Signal_Set_Phase.emit(3)

		# 5.0 Crosshair - DONE - SCALED
		# width, height
		self.crosshair = [1.0,1.0]
		width = self.rPixel[0]*self.crosshair[0]
		height = self.rPixel[1]*self.crosshair[1]
		self.crosshair = CrosshairWidget(width,height, self)
		#self.crosshair.Signal_Crosshair_Type.emit(1) # change Crosshair type
		self.crosshair.move(self.OperatorGUI_Window_Size.width()/2 - self.crosshair.width/2,
		   self.OperatorGUI_Window_Size.height()/2 - self.crosshair.height/2)
		self.crosshair.repaint()

		# Rover Tilt Widget - DONE - SCALED
		# width, height, margin
		self.roverTiltWidget_ratio = [4.0,4.0]
		width = self.rPixel[0]*self.roverTiltWidget_ratio[0]
		height = self.rPixel[1]*self.roverTiltWidget_ratio[1]
		self.roverTiltWidget = RoverTiltWidget(width,height, 5, self)
		self.roverTiltWidget.move(self.minimapWidget.width, self.OperatorGUI_Window_Size.height()-self.roverTiltWidget.height)
		#self.roverTiltWidget.repaint()

		self.GPSDisplayWidget_ratio = [8.0,0.8]
		width = self.rPixel[0]*self.GPSDisplayWidget_ratio[0]
		height = self.rPixel[1]*self.GPSDisplayWidget_ratio[1]
		font = 8
		self.GPSDpWidget = GPSDisplayWidget(width,height,font, self)
		self.GPSDpWidget.move(self.minimapWidget.width+self.roverTiltWidget.width,self.OperatorGUI_Window_Size.height()-self.GPSDpWidget.height)

		self.signalQualityWidget_ratio = [8.0,0.8]
		width = self.rPixel[0]*self.signalQualityWidget_ratio[0]
		height = self.rPixel[1]*self.signalQualityWidget_ratio[1]
		font = 8
		self.signalQualityWidget = SignalQualityWidget(width,height,font, self)
		self.signalQualityWidget.move(self.OperatorGUI_Window_Size.width()-self.driveBarWidget.width-self.signalQualityWidget.width,self.OperatorGUI_Window_Size.height()-self.signalQualityWidget.height)
		#self.signalQualityWidget

		self.astronautBoxWidget_ratio = [5.0,0.8]
		width = self.rPixel[0]*self.astronautBoxWidget_ratio[0]
		height = self.rPixel[1]*self.astronautBoxWidget_ratio[1]
		font = 8
		self.astronautBoxWidget = AstronautBoxWidget(width,height,font, self)
		self.astronautBoxWidget.move(self.OperatorGUI_Window_Size.width()-self.astronautBoxWidget.width,self.OperatorGUI_Window_Size.height()-self.driveBarWidget.height-self.astronautBoxWidget.height)

		#optional widgets
		width = self.OperatorGUI_Window_Size.width()
		length = self.OperatorGUI_Window_Size.width()
		fontSize = 10
		self.helpWidget = HelpWidget(self.OperatorGUI_Window_Size.width()/4, self.OperatorGUI_Window_Size.height(),fontSize, self)
		self.helpWidget.move(0,0)
		self.helpWidget.hide()

		# self.graphWidget = GraphWidget(14.0, 200,200,self)
		# self.graphWidget.move(self.OperatorGUI_Window_Size.width()-200, 200)
		# self.graphWidget.Signal_Add_Point.emit(1.0)
		# self.graphWidget.Signal_Add_Point.emit(11.0)
		# self.graphWidget.Signal_Add_Point.emit(5.0)
		# self.graphWidget.Signal_Add_Point.emit(8.0)
		# self.graphWidget.Signal_Add_Point.emit(6.0)
		# self.graphWidget.Signal_Add_Point.emit(9.0)
		# self.graphWidget.Signal_Add_Point.emit(2.0)
		# self.graphWidget.Signal_Add_Point.emit(4.0)

		# 5.closing stuff
		self.Signal_OperatorGUI_Quit.connect(self.Slot_OperatorGUI_Quit)
		# self.screenVideo.show()
		# self.screenVideo.streamer.run() #start video

		#refresh default nav point
		self.minimapWidget.Signal_Update_Camera_Heading.emit(0,self.rover_Telemetry_Camera_Heading)
		self.roverTiltWidget.Signal_Rotate_Base.emit(self.rover_Telemetry_Camera_Heading)
		self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		self.ROSComms.Signal_ALES_PTZ.connect(self._Slot_ALES_PTZ)
		self.ROSComms.Signal_ALES_Conveyor.connect(self._Slot_ALES_Conveyor)
		self.ROSComms.Signal_ALES_Drive.connect(self._Slot_ALES_Drive)
		self.ROSComms.Signal_ALES_GPS.connect(self._Slot_ALES_GPS)
		self.ROSComms.Signal_ALES_Signals.connect(self._SLot_ALES_Signals)
		self.ROSComms.Signal_BASE_Ping.connect(self._Slot_BASE_Ping)
		self.ROSComms.Signal_ALES_Compass.connect(self._Slot_ALES_Compass)
		self.ROSComms.Signal_Base_Conveyor.connect(self._Slot_Base_Conveyor)
		self.ROSComms.Signal_Base_Drive.connect(self._Slot_Base_Drive)
		self.ROSComms.Signal_Base_Timer.connect(self._Slot_Base_Timer)
		self.ROSComms.Signal_Base_Navpt_mirror.connect(self._Slot_Base_Navpt_mirror)
		self.ROSComms.Signal_Base_Navpt_refresh.connect(self._Slot_Base_Navpt_refresh)
		self.ROSComms.Signal_Base_Navroute_mirror.connect(self._Slot_Base_Navroute_mirror)
		self.ROSComms.Signal_Base_Navroute_refresh.connect(self._Slot_Base_Navroute_refresh)
		self.ROSComms.Signal_Base_Task_mirror.connect(self._Slot_Base_Task_mirror)
		self.ROSComms.Signal_Base_Task_refresh.connect(self._Slot_Base_Task_refresh)
		self.ROSComms.Signal_OperatorGUI_Status.connect(self._Slot_OperatorGUI_Status)
		self.ROSComms.Signal_ALES_AstronautBox_Trigger.connect(self._Slot_ALES_AstronautBox_Trigger)
		#self.ROSComms.start()
		self.minimapWidget.raise_()
		self.rover_Telemetry_GPS_longitude  = self.minimapWidget._midCoord[0]#x; init as middle of map
		self.rover_Telemetry_GPS_latitude   = self.minimapWidget._midCoord[1]#y; init as middle of map
		self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		self.videoControlWidget.Signal_MenuCamera_Set.emit("video0")

		#OPTIONAL
		#add nav points
		# for i in range(len(self.nav_points)):
		# 	self.add_nav_point(self.nav_points[i][0],self.nav_points[i][1], self.nav_points[i][2])
		# for i in range(len(self.nav_routes)):
		# 	self.add_nav_route(self.nav_routes[i][0][0],self.nav_routes[i][0][1], self.nav_routes[i][1][0],self.nav_routes[i][1][1])

	def Slot_Trigger_New_Frame(self, index):
		if self.videoDisplayWidget._cameraIndex == index:
			if self.videoDisplayWidget.currentVideoStatus == 3:
				#print 'frame1'
				self.videoDisplayWidget.setPixmap(self.pipelines._pixmaps[self.videoDisplayWidget._cameraIndex])
				self.videoDisplayWidget.repaint()
		if self.videoDisplayWidget2._cameraIndex == index:#PIP
			if self.videoDisplayWidget2.currentVideoStatus == 3:
				#print 'frame2'
				self.videoDisplayWidget2.setPixmap(self.pipelines._pixmaps[self.videoDisplayWidget2._cameraIndex])
				self.videoDisplayWidget2.repaint()


	def _Slot_ALES_PTZ(self, rotation, angle):
		self.setCameraHeading(0,rotation)
		self.setCameraAngle(0,angle)
	def _Slot_ALES_Conveyor(self, front_conveyor,back_conveyor,vertical):
		#print 'slot conveyor',front_conveyor,back_conveyor,vertical
		pass
	def _Slot_ALES_Drive(self, left,right):
		#print 'slot drive',left,right
		self.driveBarWidget.Signal_Update_Speed.emit(0, (left/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_Speed.emit(2, (left/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_Speed.emit(1, (right/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_Speed.emit(3, (right/65278.0)*100.0)
		self.driveBarWidget.repaint()
		pass
	def _Slot_ALES_GPS(self, longitude,latitude):
		self.rover_Telemetry_GPS_longitude = longitude
		self.rover_Telemetry_GPS_latitude = latitude
		self.updateGPSWidgets(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		pass
	def _Slot_ALES_Compass(self,heading):
		#print 'got Compass2',heading
		#self.setRoverHeading(self.rover_Telemetry_Rover_Heading)
		#self.setCameraHeading(0,self.rover_Telemetry_Camera_Heading)
		self.setRoverHeading(heading)
		cameraHeading = self.minimapWidget._rover_heading+self.roverTiltWidget.rotation
		if cameraHeading >= 360:
			cameraHeading -= 360
		self.compassCamera.Signal_Update_Heading.emit(cameraHeading)
		#self.setCameraHeading(0,heading)
		# self.compass.Signal_Update_Heading.emit(heading)
		# self.minimapWidget.Signal_Update_Rover_Heading.emit(heading)
		# self.minimapWidget.Signal_Repaint.emit()
		pass
	def _Slot_BASE_Ping(self, pingTime):
		#print 'got Ping2',pingTime
		self.signalQualityWidget.Signal_Update_Ping.emit(pingTime)
		self.signalQualityWidget.repaint()
	def _SLot_ALES_Signals(self, signalStr,levelStr):
		#print 'got Signals',signalStr,levelStr
		self.signalQualityWidget.Signal_Update_Link_Quality.emit(signalStr)
		self.signalQualityWidget.Signal_Update_Signal_Strength.emit(levelStr)
		self.signalQualityWidget.repaint()

	def _Slot_Base_Conveyor(self):
		pass
	def _Slot_Base_Drive(self, left,right):
		self.driveBarWidget.Signal_Update_SentSpeed.emit(0, (left/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_SentSpeed.emit(2, (left/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_SentSpeed.emit(1, (right/65278.0)*100.0)
		self.driveBarWidget.Signal_Update_SentSpeed.emit(3, (right/65278.0)*100.0)
		self.driveBarWidget.repaint()
		pass
	def _Slot_Base_Timer(self, timer):
		print 'timer2:',timer
		self.tasklistWidget.Signal_Update_Time.emit(timer)
		self.tasklistWidget.repaint()
		pass
	def _Slot_Base_Navpt_mirror(self, longitude,latitude,tag):
		self.add_nav_point(longitude,latitude,tag)
		self.HUDOverlay.repaint()
		self.setCameraHeading(0,self.rover_Telemetry_Camera_Heading)
		pass
	def _Slot_Base_Navpt_refresh(self, navptList):
		#print 'navpt refresh2'
		self.clear_nav_point()
		for i in range(len(navptList.list)):
			longitude = navptList.list[i].longitude
			latitude = navptList.list[i].latitude
			tag = navptList.list[i].tag
			self.add_nav_point(longitude,latitude,tag)
		self.HUDOverlay.repaint()
		self.setCameraHeading(0,self.rover_Telemetry_Camera_Heading)
		pass
	def _Slot_Base_Navroute_mirror(self, longitude1,latitude1,tag1, longitude2,latitude2,tag2):
		self.add_nav_route(longitude1,latitude1, longitude2,latitude2)
		pass
	def _Slot_Base_Navroute_refresh(self, navrouteList):
		#print 'navroute refresh2'
		self.clear_nav_route()
		for i in range(len(navrouteList.list)):
			longitude1 = navrouteList.list[i].pt1.longitude
			latitude1 = navrouteList.list[i].pt1.latitude
			longitude2 = navrouteList.list[i].pt2.longitude
			latitude2 = navrouteList.list[i].pt2.latitude
			self.add_nav_route(longitude1,latitude1, longitude2,latitude2)
		pass
	def _Slot_Base_Task_mirror(self, isDone):
		if isDone == True:#done current task, goto next
			self.tasklistWidget.Signal_Task_Done.emit(self.tasklistWidget._current_task)
			self.tasklistWidget.repaint()
		else:
			self.tasklistWidget.Signal_Task_Not_Done.emit(self.tasklistWidget._current_task)
			self.tasklistWidget.repaint()
		pass
	def _Slot_Base_Task_refresh(self, taskList):
		#print 'task list recv:',len(taskList.list)
		self.tasklistWidget.Signal_Clear_Tasks.emit()
		for i in range(len(taskList.list)):
			tag = taskList.list[i].tag
			task = taskList.list[i].task
			time = taskList.list[i].time
			isDone = taskList.list[i].isDone
			self.tasklistWidget.Signal_Add_Task.emit(tag,task,time,isDone)
		self.tasklistWidget.repaint()
		pass
	def _Slot_ALES_AstronautBox_Trigger(self, speed1,speed2,speed3,speed4,speed5):
		self.astronautBoxWidget.Signal_Dump_Astrobox.emit(0, speed1)
		self.astronautBoxWidget.Signal_Dump_Astrobox.emit(1, speed2)
		self.astronautBoxWidget.Signal_Dump_Astrobox.emit(2, speed3)
		self.astronautBoxWidget.Signal_Dump_Astrobox.emit(3, speed4)
		self.astronautBoxWidget.Signal_Dump_Astrobox.emit(4, speed5)

	def _Slot_OperatorGUI_Status(self, status):
		print 'get status',status
		'''
		int8 cameraNum
		int8 cameraStatus
		int8 PIPNum
		int8 PIPStatus
		int8 toggleHUD
		int8 expandMap
		int8 mapStatus
		int8 toggleHelp
		'''
		cameraNum = status.cameraNum
		cameraStatus = status.cameraStatus
		PIPNum = status.PIPNum
		PIPStatus = status.PIPStatus
		toggleHUD = status.toggleHUD
		expandMap = status.expandMap
		mapStatus = status.mapStatus
		toggleHelp = status.toggleHelp
		if cameraNum == -2: # prev camera (left DPad)
			print 'prev menu cam'
			self.prevMenuCamera()
			pass
		elif cameraNum == -1: #next camera (right DPad)
			print 'next menu cam'
			self.nextMenuCamera()
			pass
		elif cameraNum == 0: #default
			pass
		else: # set specific camera
			print 'set menu cam'
			pass
		if cameraStatus == 0:# no msg(default)
			pass
		elif cameraStatus == 1:# play?(alternate)
			print 'set cameraStatus'
			if self.videoDisplayWidget._cameraIndex != self.currentMenuCamera:#change to proper camera
				print 'change camera',self.videoDisplayWidget._cameraIndex,self.currentMenuCamera
				self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 0)
				self.videoDisplayWidget._cameraIndex = self.currentMenuCamera
				self.setCamera(self.videoDisplayWidget._cameraIndex)
			elif self.videoDisplayWidget._cameraIndex == self.currentMenuCamera:
				if self.videoDisplayWidget.currentVideoStatus == 0: # Stop: set to play
					self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 3)
					print 'play video'
				elif self.videoDisplayWidget.currentVideoStatus == 1: # pause: set to play
					self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 3)
					print 'play video'
				elif self.videoDisplayWidget.currentVideoStatus == 3: # play: set to pause
					self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 1)
					print 'pause video'
			pass
		if PIPStatus == 1:# play?(alternate)
			print 'set cameraStatus2'
			if self.videoDisplayWidget2._cameraIndex != self.currentMenuCamera:#change to proper camera
				print 'change camera2',self.videoDisplayWidget2._cameraIndex,self.currentMenuCamera
				self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 0)
				self.videoDisplayWidget2._cameraIndex = self.currentMenuCamera
				self.setPIP(self.videoDisplayWidget2._cameraIndex)
			elif self.videoDisplayWidget2._cameraIndex == self.currentMenuCamera:
				if self.videoDisplayWidget2.currentVideoStatus == 0: # Stop: set to play
					self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 3)
					print 'play video2'
				elif self.videoDisplayWidget2.currentVideoStatus == 1: # pause: set to play
					self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 3)
					print 'play video2'
				elif self.videoDisplayWidget2.currentVideoStatus == 3: # play: set to pause
					self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 1)
					print 'pause video2'
			
		if expandMap == 1:
			print 'expand map'
			self.expandMiniMap()

		pass

	def prevMenuCamera(self):
		if 1 <= self.currentMenuCamera:
			self.currentMenuCamera -= 1
			self.videoControlWidget.Signal_MenuCamera_Set.emit("video"+str(self.currentMenuCamera))
			#self.videoControlWidget.repaint()
			print 'menu prev',self.currentMenuCamera
	def nextMenuCamera(self):
		if self.currentMenuCamera < self.pipelines.numberCameras-1:
			#print 'menu next'
			self.currentMenuCamera += 1
			self.videoControlWidget.Signal_MenuCamera_Set.emit("video"+str(self.currentMenuCamera))
			#self.videoControlWidget.repaint()
			print 'menu next',self.currentMenuCamera
	def setMenuCamera(self, index):
		if 1 <= self.currentMenuCamera < self.pipelines.numberCameras-1:
			self.currentMenuCamera = index
			self.videoControlWidget.Signal_MenuCamera_Set.emit("video"+str(self.currentMenuCamera))
			#self.videoControlWidget.repaint()

	def prevCamera(self):
		if 1 <= self.videoDisplayWidget._cameraIndex:
			self.videoDisplayWidget.Signal_Camera_Set.emit(self.videoDisplayWidget._cameraIndex-1)
			self.videoControlWidget.Signal_Camera_Set.emit("video"+str(self.videoDisplayWidget._cameraIndex))
	def nextCamera(self):
		if self.videoDisplayWidget._cameraIndex < self.pipelines.numberCameras-1:
			self.videoDisplayWidget.Signal_Camera_Set.emit(self.videoDisplayWidget._cameraIndex+1)
			self.videoControlWidget.Signal_Camera_Set.emit("video"+str(self.videoDisplayWidget._cameraIndex))
	def setCamera(self, num):
		print 'set Camera(current)', self.videoDisplayWidget._cameraIndex
		if 0 <= num <= self.pipelines.numberCameras-1:
			self.videoDisplayWidget._cameraIndex = num
			self.videoDisplayWidget.Signal_Camera_Set.emit(num)
			self.videoControlWidget.Signal_Camera_Set.emit("video"+str(num))
			self.videoControlWidget.repaint()
			#self.setCameraHeading(0, self.cameraFOV[num])
	def setPIPCamera(self, num):
		print 'set PIP(current)', self.videoDisplayWidget2._cameraIndex
		if 0 <= num <= self.pipelines.numberCameras-1:
			self.videoDisplayWidget2._cameraIndex = num
			self.videoDisplayWidget2.Signal_Camera_Set.emit(num)
			self.videoControlWidget2.Signal_Camera_Set.emit("video"+str(num))
			self.videoControlWidget2.repaint()
			
	def changeCameraStatus(self, index, num):
		self.videoDisplayWidget.currentVideoStatus = num
		self.videoControlWidget.Signal_Camera_Status.emit(num)
		if self.videoDisplayWidget.currentVideoStatus <= num and self.videoDisplayWidget2.currentVideoStatus <= num:
			self.pipelines.Signal_Set_Camera().emit(index, num)
	def changePIPStatus(self, index, num):
		self.videoDisplayWidget2.currentVideoStatus = num
		self.videoControlWidget2.Signal_Camera_Status.emit(num)
		if self.videoDisplayWidget.currentVideoStatus <= num and self.videoDisplayWidget2.currentVideoStatus <= num:
			self.pipelines.Signal_Set_Camera().emit(index, num)
	# def changePIPStatus(self, num):
	# 	self.PIPWidget.Signal_Set_Camera.emit(num)
	# 	self.videoControlWidget2.Signal_Camera_Status.emit(num)
	def expandMiniMap(self):
		if not self.isMinimapLarge:
			self.bigmapWidth = self.OperatorGUI_Window_Size.height()
			self.bigmapHeight = self.OperatorGUI_Window_Size.height()
			self.minimapWidget.move(self.OperatorGUI_Window_Size.width()/2-self.bigmapWidth/2,self.OperatorGUI_Window_Size.height()/2-self.bigmapWidth/2)
			self.minimapWidget.Signal_Set_Size.emit(self.bigmapWidth,self.bigmapHeight)
			self.minimapWidget.resizeMap()
			self.minimapWidget.mapOpaque(1)
			self.isMinimapLarge = True
		else:
			#self.minimapWidget_ratio = [7.0,7.0]
			width = self.rPixel[0]*self.minimapWidget_ratio[0]
			height = self.rPixel[1]*self.minimapWidget_ratio[1]
			self.minimapWidget.Signal_Set_Size.emit(width,height)
			self.minimapWidget.move(0, self.OperatorGUI_Window_Size.height()-self.minimapWidget.height)
			self.minimapWidget.resizeMap()
			self.minimapWidget.mapOpaque(1)
			self.isMinimapLarge = False
















	# Signals Functions
	def Slot_OperatorGUI_Quit(self):
		self.close()
	def Slot_Base_Timer(self, time):
		self.base_Telemetry_Timer = time
		self.timer.Signal_OperatorGUI_TimerWidget.emit(self.base_Telemetry_Timer)
		self.tasklistWidget.Signal_Update_Time.emit(self.base_Telemetry_Timer)
	def Slot_Update_Telemetry_IMU(self, param, value):
		self.rover_Telemetry_IMU[param] = value
	def Slot_Update_Telemetry_Compass(self, heading):
		self.rover_Telemetry_Compass = heading
		self.compass.Signal_Update_Heading.emit(heading)
		self.roverTiltWidget.Signal_Rotate_Base.emit(heading)
		self.HUDOverlay.Signal_Move_Camera_X.emit(heading)
	def Slot_Update_Telemetry_Voltages(self, param, value):
		# FL/FR/RL/RR/Main = 0/1/2/3/4
		self.rover_Telemetry_Voltages[param] = value
		self.roverGraphicWidget.Signal_Update_Values.emit(param,0,value)
	def Slot_Update_Telemetry_Currents(self, param, value):
		# FL/FR/RL/RR/Main = 0/1/2/3/4
		self.rover_Telemetry_Currents[param] = value
		self.roverGraphicWidget.Signal_Update_Values.emit(param,1,value)
	def Slot_Update_Telemetry_Velocities(self, param, value):
		# FL/FR/RL/RR = 0/1/2/3
		self.rover_Telemetry_Velocities[param] = value
		self.roverGraphicWidget.Signal_Update_Values.emit(param,2,value)
		self.driveBarWidget.Signal_Update_Speed.emit(param, value)
	def Slot_Update_Telemetry_GPS(self, value):
		self.rover_Telemetry_GPS = value
	def Slot_Update_Telemetry_TiltX(self, value):
		self.roverTiltWidget.Signal_Update_TiltX.emit(value)
	def Slot_Update_Telemetry_TiltY(self, value):
		self.roverTiltWidget.Signal_Update_TiltY.emit(value)
	def Slot_Update_Telemetry_Signal_Strength(self, value):
		self.rover_Telemetry_Signal_Strength = value

	def keyPressEvent(self, e):
		if e.key() == Qt.Key_Escape: # Esc
			self.close()
		elif e.key() == Qt.Key_F1:
			#self.pipeline.set_state(gst.STATE_PLAYING)
			# self.pipelines.Signal_Set_Camera().emit(0,3)
			# self.videoControlWidget.Signal_Camera_Status.emit(3)
			#self.videoControlWidget.Signal_Camera_Status.emit(3)
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 3)
		elif e.key() == Qt.Key_F2:
			#self.pipeline.set_state(gst.STATE_PAUSED)
			# self.pipelines.Signal_Set_Camera().emit(0,1)
			# self.videoControlWidget.Signal_Camera_Status.emit(1)
			#self.videoControlWidget.Signal_Camera_Status.emit(1)
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 1)
		elif e.key() == Qt.Key_F3:
			#self.pipeline.set_state(gst.STATE_NULL)
			# self.pipelines.Signal_Set_Camera().emit(0,0)
			# self.videoControlWidget.Signal_Camera_Status.emit(0)
			#self.videoControlWidget.Signal_Camera_Status.emit(0)
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 0)
		elif e.key() == Qt.Key_F4:
			self.changeCameraStatus(self.videoDisplayWidget._cameraIndex, 0)
			self.videoDisplayWidget._cameraIndex = self.currentMenuCamera
			self.setCamera(self.videoDisplayWidget._cameraIndex)
		elif e.key() == Qt.Key_F5:
			self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 3)
		elif e.key() == Qt.Key_F6:
			self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 1)
		elif e.key() == Qt.Key_F7:
			self.changePIPStatus(self.videoDisplayWidget2._cameraIndex, 0)
		elif e.key() == Qt.Key_F8:
			self.changeCameraStatus(self.videoDisplayWidget2._cameraIndex, 0)
			self.videoDisplayWidget2._cameraIndex = self.currentMenuCamera
			self.setPIPCamera(self.videoDisplayWidget2._cameraIndex)
		elif e.key() == Qt.Key_F9:
			self.prevMenuCamera()
		elif e.key() == Qt.Key_F10:
			self.nextMenuCamera()
		elif e.key() ==  Qt.Key_F12:
			date = datetime.now()
			filename = date.strftime('%Y-%m-%d_%H-%M-%s.jpg')
			picture = QPixmap.grabWindow(self.winId())
			picture.save(filename, 'jpg')
			print 'took picture!'
		elif e.key() ==  Qt.Key_Equal:# plus;zoom in
			self.minimapWidget.zoomDecrease(0.1)
		elif e.key() ==  Qt.Key_Minus:# minus;zoom out
			self.minimapWidget.zoomIncrease(0.1)
		elif e.key() == Qt.Key_G:
			self.expandMiniMap()
		elif e.key() == Qt.Key_N:
			self.togglePanorama()
		elif e.key() == Qt.Key_M:
			if self.map_type < len(self.minimapWidget._topoQuads)-1:
				self.map_type += 1
			elif self.map_type == len(self.minimapWidget._topoQuads)-1:
				self.map_type = 0
			self.minimapWidget.Signal_Update_Map_Type.emit(self.map_type)
		elif e.key() == Qt.Key_C: #switch between task list columns
			self.tasklistWidget.Signal_Set_Phase.emit(self.showColumns)
			if self.showColumns <= 1:
				self.showColumns = 3
			else:
				self.showColumns -= 1
			print 'change phase:', self.showColumns,self.tasklistWidget.width
		elif e.key() == Qt.Key_Slash:
			if self.helpWidget.isVisible():
				self.helpWidget.hide()
			else:
				self.helpWidget.show()
		elif e.key() ==  Qt.Key_Backslash:
			self.toggleHUD()
#----------------------------------------------------------------------------------
		# elif e.key() == Qt.Key_V: # switch between text and picture for MessageWidget
		# 	self.msgWidget.Signal_Setting_isDrawText.emit(self.msgIsText)
		# 	self.msgIsText = not self.msgIsText
		# elif e.key() == Qt.Key_Z: #done current task
		# 	self.tasklistWidget.Signal_Task_Done.emit(self.tasklistWidget._current_task)
		# 	self.tasklistWidget.repaint()
		# 	self.currentTask += 1
		# elif e.key() == Qt.Key_X: #undo done current task
		# 	self.tasklistWidget.Signal_Task_Not_Done.emit(self.tasklistWidget._current_task)
		# 	self.tasklistWidget.repaint()
		# 	self.currentTask -= 1
		elif e.key() == Qt.Key_W:#Up
			self.rover_Telemetry_GPS_latitude += self.minimapWidget._PIXEL_COORD[0][1] #0.00002194137265#math.sin(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_Y
			#self.rover_Telemetry_GPS_longitude += 0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.updateGPSWidgets(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_View_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		# 	# self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		elif e.key() == Qt.Key_S:#Down
			self.rover_Telemetry_GPS_latitude -= self.minimapWidget._PIXEL_COORD[0][1] #0.00002194137265#*math.sin(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_Y
			#self.rover_Telemetry_GPS_longitude -= 0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.updateGPSWidgets(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_View_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		# 	# self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		elif e.key() == Qt.Key_A:#Left
			if self.rover_Telemetry_Rover_Heading <= 0:
				self.rover_Telemetry_Rover_Heading = 359
			else:
				self.rover_Telemetry_Rover_Heading -= 1
			self.setRoverHeading(self.rover_Telemetry_Rover_Heading)
			#self.minimapWidget.Signal_Update_Rover_Heading.emit(self.rover_Telemetry_Rover_Heading)
			# if self.rover_Telemetry_Camera_Heading <= 0:
			# 	self.rover_Telemetry_Camera_Heading = 359
			# else:
			# 	self.rover_Telemetry_Camera_Heading -= 1
			# self.minimapWidget.Signal_Update_Camera_Heading.emit(0,self.rover_Telemetry_Camera_Heading)
			# self.roverTiltWidget.Signal_Rotate_Base.emit(self.rover_Telemetry_Camera_Heading)
			# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		elif e.key() == Qt.Key_D:#Right
			if self.rover_Telemetry_Rover_Heading >= 359:
				self.rover_Telemetry_Rover_Heading = 0
			else:
				self.rover_Telemetry_Rover_Heading += 1
			self.setRoverHeading(self.rover_Telemetry_Rover_Heading)
			#self.minimapWidget.Signal_Update_Rover_Heading.emit(self.rover_Telemetry_Rover_Heading)
			# if self.rover_Telemetry_Camera_Heading >= 359:
			# 	self.rover_Telemetry_Camera_Heading = 0
			# else:
			# 	self.rover_Telemetry_Camera_Heading += 1
			# self.minimapWidget.Signal_Update_Camera_Heading.emit(0,self.rover_Telemetry_Camera_Heading)
			# self.roverTiltWidget.Signal_Rotate_Base.emit(self.rover_Telemetry_Camera_Heading)
			# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		elif e.key() == Qt.Key_Q:#Left Strafe
			self.rover_Telemetry_GPS_longitude -= self.minimapWidget._PIXEL_COORD[0][0] #0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.updateGPSWidgets(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_View_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		# 	# self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		elif e.key() == Qt.Key_E:#Right Strafe
			self.rover_Telemetry_GPS_longitude += self.minimapWidget._PIXEL_COORD[0][0] #0.00002787068004#*math.cos(90-self.rover_Telemetry_Rover_Heading) #_PIXEL_X
			self.updateGPSWidgets(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_View_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.minimapWidget.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		# 	# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		# 	# self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# elif e.key() == Qt.Key_Up:
		# 	print 'up'
		# elif e.key() == Qt.Key_Down:
		# 	print 'down'
		elif e.key() == Qt.Key_Up:
			if self.rover_Telemetry_Camera_Angle < 90:
				self.rover_Telemetry_Camera_Angle += 1
			self.setCameraAngle(0,self.rover_Telemetry_Camera_Angle)
		elif e.key() == Qt.Key_Down:
			if self.rover_Telemetry_Camera_Angle > -90:
				self.rover_Telemetry_Camera_Angle -= 1
			self.setCameraAngle(0,self.rover_Telemetry_Camera_Angle)
		elif e.key() == Qt.Key_Left:
			if self.rover_Telemetry_Camera_Heading <= 0:
				self.rover_Telemetry_Camera_Heading = 359
			else:
				self.rover_Telemetry_Camera_Heading -= 1
			self.setCameraHeading(0,self.rover_Telemetry_Camera_Heading)
		elif e.key() == Qt.Key_Right:
			if self.rover_Telemetry_Camera_Heading >= 359:
				self.rover_Telemetry_Camera_Heading = 0
			else:
				self.rover_Telemetry_Camera_Heading += 1
			self.setCameraHeading(0,self.rover_Telemetry_Camera_Heading)
		#elif e.key() == Qt.Key_Tab:

	# override normal close; append widget close(with gstreamer closing end)
	def closeEvent(self, event):
		#self.screenVideo.streamer.quit()
		event.accept()
		sys.exit()

	def updateGPSWidgets(self,longNum,latiNum):
		# self.minimapWidget.Signal_View_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# self.minimapWidget.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		# self.HUDOverlay.Signal_Update_Rover_GPS.emit(self.rover_Telemetry_GPS_longitude,self.rover_Telemetry_GPS_latitude)
		# self.HUDOverlay.Signal_Move_Camera_X.emit(self.rover_Telemetry_Camera_Heading)
		# self.GPSDpWidget.Signal_Update_GPS.emit(self.rover_Telemetry_GPS_longitude, self.rover_Telemetry_GPS_latitude)
		self.minimapWidget.Signal_View_GPS.emit(longNum, latiNum)
		self.minimapWidget.Signal_Update_Rover_GPS.emit(longNum, latiNum)
		self.HUDOverlay.Signal_Update_Rover_GPS.emit(longNum,latiNum)
		self.HUDOverlay.Signal_Move_Camera_X.emit(self.HUDOverlay._camera_heading)
		self.GPSDpWidget.Signal_Update_GPS.emit(longNum, latiNum)

	def add_nav_point(self, longitude, latitude, tag):
		self.HUDOverlay.Signal_Insert_Nav_Point.emit(longitude,latitude, tag)
		self.minimapWidget.Signal_Nav_Point.emit(longitude,latitude,tag)
		self.HUDOverlay.repaint()
		self.minimapWidget.repaint()
	def add_nav_route(self, longitude,latitude, longitude2, latitude2):
		self.minimapWidget.Signal_Nav_Route.emit(longitude,latitude, longitude2, latitude2)
		self.minimapWidget.repaint()
	def setCameraHeading(self, index, heading):
		self.minimapWidget.Signal_Update_Camera_Heading.emit(index, heading)
		if index == 0:
			self.roverTiltWidget.Signal_Rotate_Base.emit(heading)
			self.HUDOverlay.Signal_Move_Camera_X.emit(heading)
			cameraHeading = self.minimapWidget._rover_heading+heading
			if cameraHeading >= 360:
				cameraHeading -= 360
			self.compassCamera.Signal_Update_Heading.emit(cameraHeading)
	def setCameraAngle(self, index, angle):
		self.minimapWidget.Signal_Update_Camera_Angle.emit(index, angle)
		if index == 0:
			self.roverTiltWidget.Signal_Rotate_Angle.emit(angle)
			self.HUDOverlay.Signal_Move_Camera_Y.emit(angle)
			pass
	def setRoverHeading(self, heading):
		self.minimapWidget.Signal_Update_Rover_Heading.emit(heading)
		self.compass.Signal_Update_Heading.emit(heading)
	def toggleHUD(self):
		if self.showHUD:
			self.videoControlWidget.hide()
			self.videoControlWidget2.hide()
			self.HUDOverlay.hide()
			self.driveBarWidget.hide()
			self.minimapWidget.hide()
			self.compass.hide()
			self.compassCamera.hide()
			self.tasklistWidget.hide()
			self.crosshair.hide()
			self.roverTiltWidget.hide()
			self.GPSDpWidget.hide()
			self.signalQualityWidget.hide()
			self.astronautBoxWidget.hide()
			self.showHUD = False
		else:
			self.videoControlWidget.show()
			self.videoControlWidget2.show()
			self.HUDOverlay.show()
			self.driveBarWidget.show()
			self.minimapWidget.show()
			self.compass.show()
			self.compassCamera.show()
			self.tasklistWidget.show()
			self.crosshair.show()
			self.roverTiltWidget.show()
			self.GPSDpWidget.show()
			self.signalQualityWidget.show()
			self.astronautBoxWidget.show()
			self.showHUD = True
			# if self.compass.x() != 0.0:
			# 	self.compass.move(0, self.OperatorGUI_Window_Size.height()-self.minimapWidget.height-self.compass.height)
	def togglePanorama(self):
		if self.compassCamera.isVisible():
			self.compassCamera.hide()
		else:
			self.compassCamera.show()
	def clear_nav_point(self):
		self.HUDOverlay._nav_points = []
		self.minimapWidget.Signal_Clear_Nav_Point.emit()

	def clear_nav_route(self):
		self.minimapWidget.Signal_Clear_Nav_Route.emit()

##############               SUPPORTING FUNCTIONS                 ##############
'''
Centers the main window on the screen.
'''
def center(widget):
	qr = widget.frameGeometry()
	cp = QDesktopWidget().availableGeometry().center()
	qr.moveCenter(cp)
	widget.move(qr.topLeft())

##############                   MAIN FUNCTION                    ##############

def main():
	#gobject.threads_init()
	#Gst.init(None)

	app = QApplication(sys.argv)	# application object for PyQT
	gui = OperatorGUI()
	gui.show()
	loop = gobject.MainLoop()
	try:
		loop.run()
		app.exec_()
	except KeyboardInterrupt:
		print "Ctrl+C pressed, exitting"
		pass

if __name__ == '__main__':
	ret = gst.element_register(VideoSink, 'VideoSink')
	print 'register:',ret
	gobject.threads_init()
	#main()