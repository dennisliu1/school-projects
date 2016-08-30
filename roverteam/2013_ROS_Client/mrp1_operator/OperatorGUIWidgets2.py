#!/usr/bin/python
# -*- coding: utf-8 -*-

import math

import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *

import pygst
pygst.require("0.10")
import gst
import gobject

import struct

hdr_fmt = "!LQQi"
hdr_len = struct.calcsize(hdr_fmt)

import rospkg
#ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
path = r.get_path('mrp1_operator')

##############                     WIDGETS                        ##############
#01.  Camera Tab Widget
class CameraTabWidget(QTabWidget):
	Signal_OperatorGUI_CameraTabWidget = pyqtSignal()
	def __init__(self, parent = None, numTabs = 10):
		super(CameraTabWidget, self).__init__(parent)
		self.setToolTip('This is the <b>CameraTab</b> widget')
		cameraTabWidget_tabSettings = []
		for i in range(numTabs):
			tabMenu = []
			tabMenu.append(QWidget(self))
			tabMenu.append(QHBoxLayout())
			tabMenu.append(QLabel('IP:Pt', self))
			tabMenu.append(QLineEdit(self))
			tabMenu.append(QPushButton('Start', self))
			for j in range(len(tabMenu)-2):
				tabMenu[1].addWidget(tabMenu[j+2])
			tabMenu[0].setLayout(tabMenu[1])
			cameraTabWidget_tabSettings.append(tabMenu)
			self.addTab(tabMenu[0], str(i))
		self.resize(self.sizeHint())

#02.  Message Box Widget - OPENCV
class MessageWidget(QLabel):
	Signal_Setting_isDrawText = pyqtSignal(bool)
	Signal_Setting_Text = pyqtSignal(str)
	Signal_Setting_Pic = pyqtSignal(str)
	def __init__(self, width, height, parent = None):
		super(MessageWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Message</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		
		self.message_text = "OperatorGUI"
		self.img = QImage('YURTLogo.png')
		self.isText=  False
		self.Signal_Setting_isDrawText.connect(self.Slot_Setting_isDrawText)
		self.Signal_Setting_Text.connect(self.Slot_Setting_Text)
		self.Signal_Setting_Pic.connect(self.Slot_Setting_Pic)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawMessage(event, qp)
		qp.end()
		
	def drawMessage(self, event, qp):
		qp.setPen(QColor(168, 34, 3))
		qp.setBrush(Qt.NoBrush)
		qp.setFont(QFont('Decorative', 20))
		if self.isText:
			qp.drawText(event.rect(), Qt.AlignCenter, self.message_text)
		else:
			img_rect = QRect(0,0, self.img.width(),self.img.height())
			qp.drawImage(event.rect(), self.img, img_rect)
		qp.drawRect(0,0, self.width-1, self.height-1)

	def Slot_Setting_isDrawText(self, bool):
		self.isText = bool
		self.repaint()
	def Slot_Setting_Text(self, text):
		self.message_text = text
		self.repaint()
	def Slot_Setting_Pic(self, pic):
		self.img = QImage(pic)
		self.repaint()

#04.  Rover Info Widget - OPENCV - Cleaned
# Edge Color,Font Color,Background Color
class RoverGraphicWidget(QWidget):
	Signal_Update_Values = pyqtSignal(int, int, float)
	def __init__(self, width, height, parent = None):
		super(RoverGraphicWidget, self).__init__(parent)
		#self.setToolTip('This is the <b>Rover Graphic</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		
		self.Signal_Update_Values.connect(self._Slot_Update_Values)
		self.color_edge = QColor(168, 34, 3)
		self.color_background = QColor(168, 100,100, 100)
		self.color_Font = QColor(168, 34, 3)

		self._baseWidth = width/15
		self._baseHeight = height/30
		self._fontSize = (self._baseWidth+self._baseHeight)/2
		self._margin = self._fontSize
		self._roverVoltages = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self._roverCurrents = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self._roverVelocities = [0.0, 0.0, 0.0, 0.0, 0.0] # FL/R, RL/R, main
		self._wheelRect = QSize(self._baseWidth*4,self._baseHeight*10)
		self._mainRect = QSize(self._baseWidth*8,self._baseHeight*20)
		self._font = QFont('Decorative', self._fontSize)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawRover(event, qp)
		qp.end()

	def _drawRover(self, event, qp):
		qp.setPen(self.color_edge)
		qp.setBrush(self.color_background)
		qp.drawRect(0,0, self._wheelRect.width(),self._wheelRect.height()) # FL Wheel
		qp.drawRect(self._wheelRect.width()+self._mainRect.width(),0, self._wheelRect.width(),self._wheelRect.height()) #FR wheel
		qp.drawRect(0,self._wheelRect.height()*2, self._wheelRect.width(),self._wheelRect.height()) #RL Wheel
		qp.drawRect(self._wheelRect.width()+self._mainRect.width(),self._wheelRect.height()*2, self._wheelRect.width(),self._wheelRect.height()) #RR Wheel
		qp.drawRect(self._wheelRect.width(),self._mainRect.height()/4, self._mainRect.width(),self._mainRect.height()) #main frame
		
		qp.setPen(self.color_edge)
		qp.setBrush(Qt.NoBrush)
		qp.setFont(self._font)
		# FL Wheel
		qp.drawText(self._margin,3*self._baseHeight, "V"+str(self._roverVoltages[0]))
		qp.drawText(self._margin,5*self._baseHeight, "C"+str(self._roverCurrents[0]))
		qp.drawText(self._margin,7*self._baseHeight, "S"+str(self._roverVelocities[0]))
		# FR Wheel
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,3*self._baseHeight, "V"+str(self._roverVoltages[1]))
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,5*self._baseHeight, "C"+str(self._roverCurrents[1]))
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,7*self._baseHeight, "S"+str(self._roverVelocities[1]))
		# RL Wheel
		qp.drawText(self._margin,self._wheelRect.height()*2+3*self._baseHeight, "V"+str(self._roverVoltages[2]))
		qp.drawText(self._margin,self._wheelRect.height()*2+5*self._baseHeight, "C"+str(self._roverCurrents[2]))
		qp.drawText(self._margin,self._wheelRect.height()*2+7*self._baseHeight, "S"+str(self._roverVelocities[2]))
		# RR Wheel
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,self._wheelRect.height()*2+3*self._baseHeight, "V"+str(self._roverVoltages[3]))
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,self._wheelRect.height()*2+5*self._baseHeight, "C"+str(self._roverCurrents[3]))
		qp.drawText(self._wheelRect.width()+self._mainRect.width()+self._margin,self._wheelRect.height()*2+7*self._baseHeight, "S"+str(self._roverVelocities[3]))
		# Main Chassis
		qp.drawText(self._wheelRect.width()+self._baseWidth+self._margin,self._mainRect.height()/2+3*self._baseHeight, "V"+str(self._roverVoltages[4]))
		qp.drawText(self._wheelRect.width()+self._baseWidth+self._margin,self._mainRect.height()/2+5*self._baseHeight, "C"+str(self._roverCurrents[4]))
		qp.drawText(self._wheelRect.width()+self._baseWidth+self._margin,self._mainRect.height()/2+7*self._baseHeight, "S"+str(self._roverVelocities[4]))
		
		qp.setBrush(Qt.NoBrush)
		qp.drawRect(0,0, self.width-1, self.height-1)

	def _Slot_Update_Values(self, param, part, value):
		if(part == 0):
			self._roverVoltages[param] = value
		elif(part == 1):
			self._roverCurrents[param] = value
		elif(part == 2):
			self._roverVelocities[param] = value
		self.repaint()

#05.  Accelerator Bar Widget - OPENCV - Cleaned
# Border, Background,ticks,bar(+/-)
class DriveBarWidget(QWidget):
	Signal_Update_Speed = pyqtSignal(int, float)
	def __init__(self, ruler_tick_size, ruler_tick_length, ruler_tick_num, large_tick_offset, parent = None):
		super(DriveBarWidget, self).__init__(parent)
		#self.setToolTip('This is the <b>Drive Bar</b> widget')
		self.ruler_tick_size = ruler_tick_size    # how far apart small ticks are
		self.ruler_tick_length = ruler_tick_length# how long each small tick is
		self.ruler_tick_num = ruler_tick_num      # number of small ticks
		self.large_tick_offset = large_tick_offset# every X small tick is a large tick
		self.tick_large_length = self.ruler_tick_length*2 # how long each large tick is
		self.width = self.ruler_tick_size*self.ruler_tick_num
		self.height = self.tick_large_length*4
		self.resize(self.width,self.height)

		self._speeds = [0.0,0.0, 0.0,0.0]
		self._speedLeft = 0
		self._speedRight = 0
		self._speedLeftPercent = 0
		self._speedRightPercent = 0
		self._MAX_WHEEL_SPEED = 100.0 #by percent
		self.Signal_Update_Speed.connect(self._Slot_Update_Speed)

		self.color_border = QColor(QColor(168, 34, 3))
		self.color_background = Qt.NoBrush
		self.color_ticks = QColor(168, 34, 3)
		self.color_barPositive = QColor(255, 175, 175)
		self.color_barNegative = QColor(175, 175, 255)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawDriveBar(event, qp)
		qp.end()
		
	def _drawDriveBar(self, event, qp):
		self._drawBorder(event,qp)
		self._drawSpeedBar(event,qp)
		self._drawTicks(event, qp)

	def _drawSpeedBar(self, event, qp):
		if self._speedLeftPercent > 0:
			qp.setPen(self.color_barPositive)
			qp.setBrush(self.color_barPositive)
		else:
			qp.setPen(self.color_barNegative)
			qp.setBrush(self.color_barNegative)
		qp.drawRect(self.width/2,0, (self.width/2)*self._speedLeftPercent, self.height/2)
		if self._speedRightPercent > 0:
			qp.setPen(self.color_barPositive)
			qp.setBrush(self.color_barPositive)
		else:
			qp.setPen(self.color_barNegative)
			qp.setBrush(self.color_barNegative)
		qp.drawRect(self.width/2,self.height/2, (self.width/2)*self._speedRightPercent, self.height)

	def _drawTicks(self, event, qp):
		qp.setPen(self.color_ticks)
		qp.setBrush(Qt.NoBrush)
		for i in range(self.ruler_tick_num):
			qp.drawLine(self.ruler_tick_size*i, 0,self.ruler_tick_size*i,self.ruler_tick_length) # top ruler
			if i%self.large_tick_offset == 0:
				qp.drawLine(self.ruler_tick_size*i, 0,self.ruler_tick_size*i,self.tick_large_length) # top ruler
			qp.drawLine(self.ruler_tick_size*i, self.height,self.ruler_tick_size*i,self.height-self.ruler_tick_length) #bottom ruler
			if i%self.large_tick_offset == 0:
				qp.drawLine(self.ruler_tick_size*i, self.height,self.ruler_tick_size*i,self.height-self.tick_large_length) #bottom ruler
	
	def _drawBorder(self, event, qp):
		qp.setPen(self.color_border)
		qp.setBrush(self.color_background)
		qp.drawLine(0,self.height/2, self.width, self.height/2) # draw middle(H) line
		qp.drawLine(self.width/2,0, self.width/2,self.height) # draw middle(V) line
		qp.drawRect(0,0, self.width-1, self.height-1)

	def _Slot_Update_Speed(self, param, value):
		self._speeds[param] = value
		if param == 0 or param == 2:
			self.speedLeftPercent = (self._speeds[0]/self._MAX_WHEEL_SPEED+self._speeds[2]/self._MAX_WHEEL_SPEED)/2.0
		else:
			self.speedRightPercent = (self._speeds[1]/self._MAX_WHEEL_SPEED+self._speeds[3]/self._MAX_WHEEL_SPEED)/2.0
		self.repaint()

#06.  Timer Widget - DONE
class TimerWidget(QLCDNumber):
	Signal_OperatorGUI_TimerWidget = pyqtSignal(int)
	def __init__(self, time, parent = None):
		super(TimerWidget, self).__init__(5, parent)
		self.time = time
		self.setToolTip('This is the <b>Timer</b> widget')
		self.Slot_TimerWidget_Update(time)
		self.Signal_OperatorGUI_TimerWidget.connect(self.Slot_TimerWidget_Update)
		
	def Slot_TimerWidget_Update(self, time):
		self.time = time
		timeStr = str(time/60)+":"+str(time%60)
		self.display(timeStr)

#08.  Compass Widget - OPENCV - Cleaned
# border,background,tick,font,nav points
class CompassWidget(QLabel):
	Signal_Update_Heading = pyqtSignal(int)
	def __init__(self, width, height, compass_tick_spacing, font_size, parent = None):
		super(CompassWidget, self).__init__(parent)
		#self.setToolTip('This is the <b>Compass</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.compass_tick_spacing = compass_tick_spacing # spacing between each tick
		self.font_size = font_size
		self.font_width = self.font_size
		
		self._compass_mini_tick_length =  int(self.height/8) # length of a small tick
		self._compass_small_tick_length = int(self.height/4) # length of a small tick
		self._compass_large_tick_length = int(self.height/2) # length of a large tick
		self._compass_small_tick_rep = 9 # what a small tick represents
		self._compass_large_tick_rep = 45# what a large tick represents
		self._compass_heading_arrow_height = int(self.height/4)
		self._compass_names = ['N','NE','E','SE','S','SW','W','NW']
		#self.ticks_showable = int(self.width/self.compass_tick_spacing) #max number of ticks showable
		self._heading = 0 # heading is counter clockwise
		self.Signal_Update_Heading.connect(self._Slot_Update_Heading)

		self._color_border = QColor(168, 34, 3)
		self._color_tick = QColor(168, 34, 3)
		self._color_background = Qt.NoBrush
		self._color_font = QColor(168, 34, 3)
		self._font = QFont('Decorative', self.font_size)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawBorder(event, qp)
		self._drawCompass(event, qp)
		qp.end()

	def _drawBorder(self, event, qp):
		qp.setPen(self._color_border)
		qp.setBrush(self._color_background)
		qp.drawRect(0,0, self.width-1, self.height-1)
		#draw triangle for center heading
		qp.drawLine(self.width/2-self.compass_tick_spacing,self.height, self.width/2,self.height-self._compass_heading_arrow_height)
		qp.drawLine(self.width/2+self.compass_tick_spacing,self.height, self.width/2,self.height-self._compass_heading_arrow_height)
		
	def _drawCompass(self, event, qp):
		self._drawSideTicks(qp, 1, self._heading)
		self._drawSideTicks(qp, -1, self._heading)
		
	def _drawSideTicks(self, qp, incrementor, heading):
		qp.setPen(self._color_tick)
		qp.setBrush(Qt.NoBrush)
		qp.setFont(self._font)
		mid_heading = heading
		current_heading = heading
		drawTicks = True
		while drawTicks: # right side ticks only
			tick_length = self._compass_mini_tick_length
			if current_heading%self._compass_large_tick_rep == 0:
				offset = int((len(self._compass_names[current_heading/self._compass_large_tick_rep])*self.font_width)/2)
				qp.setPen(self._color_font)
				qp.drawText(self.width/2+(current_heading-mid_heading)*self.compass_tick_spacing-offset, self.font_size, self._compass_names[current_heading/self._compass_large_tick_rep])
				tick_length = self._compass_large_tick_length
			elif current_heading%self._compass_small_tick_rep == 0:
				tick_length = self._compass_small_tick_length
			qp.setPen(self._color_tick)
			qp.drawLine(self.width/2+(current_heading-mid_heading)*self.compass_tick_spacing, self.height,
				        self.width/2+(current_heading-mid_heading)*self.compass_tick_spacing, self.height-tick_length)
			current_heading += incrementor # paint next tick
			if not (0 <= self.width/2+(current_heading-mid_heading)*self.compass_tick_spacing and self.width/2+(current_heading-mid_heading)*self.compass_tick_spacing <= self.width):
				drawTicks = False

	# heading is counterclockwise, so flip it
	def _Slot_Update_Heading(self, heading):
		self._heading = heading
		self.update()
		self.repaint()

#09.  Task List Widget - OPENCV - Cleaned
# Border, background, font
class TaskListWidget(QLabel):
	Signal_Update_Time = pyqtSignal(int)
	Signal_Set_Phase = pyqtSignal(int)
	Signal_Clear_Tasks = pyqtSignal()
	Signal_Add_Task = pyqtSignal(QString, QString)
	Signal_Task_Done = pyqtSignal(int)
	Signal_Task_Not_Done = pyqtSignal(int)
	def __init__(self, margin,phase1,phase2,phase3,fontBase, numRows, parent = None):
		super(TaskListWidget, self).__init__(parent)
		self.setToolTip('This is the <s>Task List</s> widget')
		self.margin = 5
		self.phase1 = phase1
		self.phase2 = phase2
		self.phase3 = phase3
		self.fontWidth = fontBase
		self.fontHeight = fontBase
		self.rows_print = numRows
		self.width = self.margin+(self.phase1+self.phase2+self.phase3)*self.fontWidth+self.margin
		self.height = self.rows_print*(self.fontHeight*1.5)
		self.resize(self.width, self.height)
		
		self._row_print_start_index = 0
		self._time = 0
		self._phase = 3
		
		self.Signal_Update_Time.connect(self._Slot_Update_Time)
		self.Signal_Set_Phase.connect(self._Slot_Set_Phase)
		self.Signal_Clear_Tasks.connect(self._Slot_Clear_Tasks)
		self.Signal_Add_Task.connect(self._Slot_Add_Task)
		self.Signal_Task_Done.connect(self._Slot_Task_Done)
		self.Signal_Task_Not_Done.connect(self._Slot_Task_Not_Done)
		self._current_task = 1 #1 = header; -1 = no header
		self._index_list = []
		self._task_list = []
		self._time_list = []
		self._is_done = []

		self._color_border = QColor(168, 34, 3)
		self._color_background = QColor(168, 34, 3, 50)
		self._color_font = QColor(168, 34, 3)
		self._font = QFont('Decorative', self.fontHeight)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawBorder(event, qp)
		self._drawMessage(event, qp)
		qp.end()

	def _drawBorder(self, event, qp):
		qp.setPen(self._color_border)
		qp.setBrush(self._color_background)
		qp.drawRect(0,0, self.width-1, self.height-1)
		
	def _drawMessage(self, event, qp):
		qp.setPen(self._color_font)
		qp.setBrush(Qt.NoBrush)
		qp.setFont(self._font)
		rows_to_print = len(self._index_list)
		if len(self._index_list) > self.rows_print:
			rows_to_print = self.rows_print
		timeStr = str(self._time/60)+":"+str(self._time%60)
		qp.drawText(self.margin+(self.phase1+self.phase2)*self.fontWidth, (self.fontHeight*1.5)*1,  timeStr)
		j = 0
		for i in range(self._row_print_start_index,self._row_print_start_index+rows_to_print):
			if self._phase >= 1:
				qp.drawText(self.margin, (self.fontHeight*1.5)*(j+1), self._index_list[i])
				if self._is_done[i]:
					qp.drawLine(self.margin, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2, self.fontHeight*2, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2)
			if self._phase >= 2:
				qp.drawText(self.fontHeight*2+self.margin, (self.fontHeight*1.5)*(j+1), self._task_list[i])
				if self._is_done[i]:
					qp.drawLine(self.fontHeight*2+self.margin, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2, 
								self.fontHeight*2+self.margin+self.fontHeight*12, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2)
			if self._phase >= 3:
				if (self._is_done[i] or i == self._current_task) and self._time_list[i] >= 0:
					qp.drawText(self.margin+(self.phase1+self.phase2)*self.fontWidth, (self.font*1.5)*(j+1), str(self._time_list[i]/60)+":"+str(self._time_list[i]%60))
			j += 1

	def _Slot_Update_Time(self, time):
		# print 'time:',time,' ',str(self._current_task),' ',self._is_done[self._current_task]
		self._time = 3600-time
		self._time_list[self._current_task] = self.time
		self.repaint()

	def _Slot_Set_Phase(self, phase):
		#print 'phase',phase
		self.phase = phase
		if self._phase == 1:
			self.width = self.margin+(self.phase1)*self.fontWidth+self.margin
		elif self._phase == 2:
			self.width = self.margin+(self.phase1+self.phase2)*self.fontWidth+self.margin
		elif self._phase == 3:
			self.width = self.margin+(self.phase1+self.phase2+self.phase3)*self.fontWidth+self.margin
		if self._current_task < 0:
			self._current_task = 0
		self.resize(self.width, self.height)
		self.repaint()

	def _Slot_Clear_Tasks(self):
		self._index_list = []
		self._task_list = []
		self._time_list = []
		self._is_done = []

	def _Slot_Add_Task(self, index, task):
		self._index_list.append(index)
		self._task_list.append(task)
		self._time_list.append(-1)
		self._is_done.append(False)

	def _Slot_Task_Done(self, index):
		# print str(index),' ',str(self._current_task)
		if index < len(self._is_done):
			self._is_done[index] = True
			self._time_list[index] = self.time
			self._current_task += 1
			#print index,len(self._is_done),len(self._time_list),self._current_task
			print index,self._is_done[index],self._time_list[index],self._current_task

	def _Slot_Task_Not_Done(self, index):
		if index > 1:
			self._current_task -= 1
			self._is_done[index-1] = False
			self._time_list[index-1] = -1
			print index,self._is_done[index-1],self._time_list[index-1],self._current_task

#10.  Crosshair - OPENCV - Cleaned
class CrosshairWidget(QWidget):
	CROSSHAIR_TYPE_CROSS = 0 #default: + sign shape
	CROSSHAIR_TYPE_CIRCLE = 1 #circle
	Signal_Crosshair_Type = pyqtSignal(int)
	def __init__(self, width,height, parent = None):
		super(CrosshairWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Crosshair</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self._crosshair_type = 0
		self.Signal_Crosshair_Type.connect(self._Slot_Crosshair_Type)

		self._color1 = QColor(255,125,125,125)
		self._color2 = (125,125,255,255)
		
	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		qp.translate(self.width/2,self.height/2)
		self._drawCrosshair(event, qp)
		qp.end()

	def _drawCrosshair(self, event, qp):
		if self._crosshair_type == 0: # draw plus sign
			qp.setPen(self._color1)
			qp.drawLine(0, self.height/2, 0, -self.height/2) #vertical line
			qp.drawLine(self.width/2, 0, -self.width/2, 0) #horizontal line
		elif self._crosshair_type == 1: #draw circle with dot
			qp.setPen(self._color1)
			qp.setBrush(self._color1)
			qp.drawEllipse(-self.width/4, -self.height/4, self.width/2, self.height/2) # big circle
			qp.setPen(self._color2)
			qp.setBrush(self._color2)
			qp.drawEllipse(-self.width/20,-self.height/20, self.width/10, self.height/10) # tiny dot
		#qp.drawRect(0,0, self.width()-1,self.height()-1)

	def _Slot_Crosshair_Type(self, ctype):
		self._crosshair_type = ctype
		self.repaint()

#11.  Rover Tilt Info Widget - OPENCV - Cleaned
class RoverTiltWidget(QLabel):
	Signal_Rotate_Base = pyqtSignal(float)
	Signal_Update_TiltX = pyqtSignal(float)
	Signal_Update_TiltY = pyqtSignal(float)
	def __init__(self, width, height, margin, parent = None):
		super(RoverTiltWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Rover Tilt</b> widget')
		self.width = width
		self.height = height
		self.margin = margin
		self.resize(self.width, self.height)
		
		self._roverPicture = self.RoverPictureWidget(self.width, self.height, (self.width-self.margin*2)/6, (self.height-self.margin*2)/4, self)
		self._roverPicture.move(0,0)
		self._roverCamera = self.RoverCameraWidget(self.width, self.height,self)
		self._roverCamera.move(0,0)
		#self.roverPicture.move(self.width/2, self.height/2)
		self.Signal_Rotate_Base.connect(self._Slot_Rotate_Base)
		self.Signal_Update_TiltX.connect(self._Slot_Update_TiltX)
		self.Signal_Update_TiltY.connect(self._Slot_Update_TiltY)
		self._tiltX = 0
		self._tiltY = 0
		self.rotation = 0

		self._color_border = QColor(168, 34, 3)
		self._color_background = QColor(168, 34, 3, 50)
		self._color_font = QColor(255, 255, 3)
		self._color_foward = QColor(95,95,255)
		self._color_rover_edge = QColor(255,125,125)
		self._font = QFont('Decorative', 12)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawBorder(event, qp)
		self._drawMessage(event, qp)
		qp.end()

	def _drawBorder(self, event, qp):
		qp.setPen(self._color_border)
		qp.setBrush(self._color_background)
		qp.drawRect(0,0, self.width-1, self.height-1)

	def _drawMessage(self, event, qp):
		qp.setPen(self._color_font)
		qp.setFont(self._font)
		qp.drawText(00, self.height-2, 'H:'+str(self.rotation))
		qp.drawText(100, self.height-2-20, 'X'+str(self._tiltX))
		qp.drawText(100, self.height-2, 'Y'+str(self._tiltY))

	def _Slot_Rotate_Base(self, rotation):
		self._roverPicture.rotateBase(rotation)
		self.rotation = rotation
		self.repaint()
	def _Slot_Update_TiltX(self, tiltX):
		self._roverPicture.update_tiltX(tiltX)
		self._tiltX = tiltX
		self.repaint()
	def _Slot_Update_TiltY(self, tiltY):
		self._roverPicture.update_tiltY(tiltY)
		self._tiltY = tiltY
		self.repaint()

	# Used by Rover Tilt Info Widget
	class RoverPictureWidget(QWidget):
		def __init__(self, width,height, CRight, CDown, parent = None):
			super(RoverTiltWidget.RoverPictureWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.CDown = CDown #35
			self.CUp = self.CDown * -1
			self.CRight = CRight #20
			self.CLeft = self.CRight * -1
			self.resize(self.width,self.height)
			self.pts = []
			self.pts.append(QPoint(0,0)) # 0mid
			self.pts.append(QPoint(self.CLeft ,self.CUp-self.CLeft)) # 1mid left
			self.pts.append(QPoint(self.CLeft ,self.CDown)) # 2bottom left
			self.pts.append(QPoint(self.CRight,self.CDown)) # 3bottom right
			self.pts.append(QPoint(self.CRight,self.CUp+self.CRight)) # 4mid right
			self.pts.append(QPoint(          0,self.CUp)) # 5top
			self.pts.append(QPoint(self.CLeft ,0)) # 6left mid
			self.pts.append(QPoint(self.CRight,0)) # 7right mid
			self.pts.append(QPoint(          0,self.CDown)) # 8bottom mid
			self.rotation = 0
			self.tiltX = 0
			self.tiltY = 0
			self.tiltX_Shorten = 0
			self.tiltY_Shorten = 0
			self.tiltX_Shorten_Max = 5
			self.tiltY_Shorten_Max = 5 #half of 1 side can be taken out for visual tilt
			self.base_color = QColor(255,125,125)
			self.quadrant_colors = []
			for i in range(4):
				self.quadrant_colors.append(QColor(255,125,125))

		def paintEvent(self, e):
			qp = QPainter()
			qp.begin(self)
			qp.translate(self.width / 2, self.height / 2)
			qp.rotate(self.rotation)
			self._drawPic(qp)
			qp.end()
		def _drawPic(self, qp):
			qp.setPen(self.parent._color_rover_edge)
			qp.setBrush(self.parent._color_rover_edge)
			qp.drawEllipse(self.pts[0],5,5)
			qp.drawLine(self.pts[1], self.pts[2])
			qp.drawLine(self.pts[2], self.pts[3])
			qp.drawLine(self.pts[3], self.pts[4])
			qp.drawLine(self.pts[1], self.pts[5])
			qp.drawLine(self.pts[4], self.pts[5])

			qp.drawLine(self.pts[0], self.pts[5]) # left line
			qp.drawLine(self.pts[0], self.pts[6]) # right line
			qp.drawLine(self.pts[0], self.pts[7]) # down line
			qp.drawLine(self.pts[0], self.pts[8]) # up line
			qp.setBrush(self.quadrant_colors[0])
			qp.drawPolygon(self.pts[1],self.pts[6],self.pts[0],self.pts[5]) #top left part
			qp.setBrush(self.quadrant_colors[1])
			qp.drawPolygon(self.pts[4],self.pts[7],self.pts[0],self.pts[5]) # top right part
			qp.setBrush(self.quadrant_colors[2])
			qp.drawPolygon(self.pts[2],self.pts[6],self.pts[0],self.pts[8]) # bottom left part
			qp.setBrush(self.quadrant_colors[3])
			qp.drawPolygon(self.pts[3],self.pts[7],self.pts[0],self.pts[8]) # bottom right part

		def rotateBase(self, rotation):
			self.rotation = -rotation
			self.repaint()
		def update_tiltX(self, tiltX):
			self.tiltX = tiltX
			self.update_tilt()
			self.repaint()
		def update_tiltY(self, tiltY):
			self.tiltY = tiltY
			self.update_tilt()
			self.repaint()
		def update_tilt(self):
			#self.tiltX/90.0
			# 90' = 125+125/2
			# 00' = 125
			#-90' = 125-125/2
			# 125 * self.tiltX/90
			#print 'tiltX:',str(self.tiltX),' ',str((self.tiltX/90.0))
			#print 'update Tilt:125+',str((self.tiltX/90.0)*(125/2)),'+',str((self.tiltY/90.0)*(125/2))
			self.tiltX_Shorten = (self.tiltX/90.0)*self.tiltX_Shorten_Max
			self.tiltY_Shorten = (self.tiltY/90.0)*self.tiltY_Shorten_Max

			# assumes: tilt left = -, tilt right = +; tilt up = +, tilt down = -
			self.quadrant_colors[0].setGreen(127+(self.tiltX/90.0)*(-125/2)+(self.tiltY/90.0)*(+125/2)) #FL
			self.quadrant_colors[0].setBlue (127+(self.tiltX/90.0)*(-125/2)+(self.tiltY/90.0)*(+125/2)) 
			self.quadrant_colors[1].setGreen(127+(self.tiltX/90.0)*(+125/2)+(self.tiltY/90.0)*(+125/2)) #FR
			self.quadrant_colors[1].setBlue (127+(self.tiltX/90.0)*(+125/2)+(self.tiltY/90.0)*(+125/2)) 
			self.quadrant_colors[2].setGreen(127+(self.tiltX/90.0)*(-125/2)+(self.tiltY/90.0)*(-125/2)) #RL
			self.quadrant_colors[2].setBlue (127+(self.tiltX/90.0)*(-125/2)+(self.tiltY/90.0)*(-125/2)) 
			self.quadrant_colors[3].setGreen(127+(self.tiltX/90.0)*(+125/2)+(self.tiltY/90.0)*(-125/2)) #RR
			self.quadrant_colors[3].setBlue (127+(self.tiltX/90.0)*(+125/2)+(self.tiltY/90.0)*(-125/2)) 

	# Used by Rover Tilt Info Widget
	class RoverCameraWidget(QWidget):
		def __init__(self, width, height, parent = None):
			super(RoverTiltWidget.RoverCameraWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.resize(self.width,self.height)

		def paintEvent(self, e):
			qp = QPainter()
			qp.begin(self)
			self._drawPic(qp)
			qp.end()

		def _drawPic(self, qp):
			qp.setPen(self.parent._color_foward)
			qp.drawLine(self.width/2-1,self.height/2, self.width/2-1,self.height/4-10)
			qp.drawLine(self.width/2-2,self.height/2, self.width/2-2,self.height/4-10)
			qp.drawLine(self.width/2,self.height/2, self.width/2,self.height/4-10)# rover camera arrow always points forward
			qp.drawLine(self.width/2+1,self.height/2, self.width/2+1,self.height/4-10)
			qp.drawLine(self.width/2+2,self.height/2, self.width/2+2,self.height/4-10)

#12.  HUD Overlay - OPENCV
# edge, background, font color
class HUDOverlayWidget(QWidget):
	Signal_Insert_Nav_Point = pyqtSignal(float,float, QString)
	Signal_Update_Rover_GPS = pyqtSignal(float,float)
	Signal_Move_Camera_X = pyqtSignal(float)
	Signal_Move_Camera_Y = pyqtSignal(float)
	def __init__(self, width,height, fov, navSize, parent = None):
		super(HUDOverlayWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Crosshair</b> widget')
		self.width = width
		self.height = height
		self.resize(width,height)
		
		self._fov_width = fov
		self._fov_height = fov
		self.navSize = navSize
		self._camera_heading = 0
		self._camera_Up = 0
		self._heading_translation_x = self.width/self._fov_width #1800/60 = 30 pixels per degree
		self._heading_translation_y = self.height/self._fov_height
		self._rover_nav = self.NavPoint(0,0,'')
		self._nav_points = [] #<long,lati,name>
		self._nav_point_Diamond = []
		self._nav_point_Diamond.append(QPoint(self.navSize,0))
		self._nav_point_Diamond.append(QPoint(0,self.navSize))
		self._nav_point_Diamond.append(QPoint(self.navSize*-1,0))
		self._nav_point_Diamond.append(QPoint(0,self.navSize*-1))
		self.Signal_Insert_Nav_Point.connect(self.Slot_Insert_Nav_Point)
		self.Signal_Update_Rover_GPS.connect(self.Slot_Update_Rover_GPS)
		self.Signal_Move_Camera_X.connect(self.Slot_Move_Camera_X)
		self.Signal_Move_Camera_Y.connect(self.Slot_Move_Camera_Y)

		self._color_edge = QColor(255,125,125, 200)
		self._color_background = QColor(255,125,125, 100)
		self._color_font = QColor(255,125,125, 200)
		self._font = QFont('times', 12)
		self.update_nav_relative()

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		qp.translate(self.width/2,self.height/2)
		self.drawHUD(qp)
		qp.end()

	def drawHUD(self, qp):
		self.drawDiamonds(qp)

	def drawDiamonds(self, qp):
		for i in range(len(self._nav_points)):
			qp.setPen(self._color_edge)
			qp.setBrush(self._color_background)
			qp.drawPolygon(self._nav_points[i].icon[0],self._nav_points[i].icon[1],
				self._nav_points[i].icon[2],self._nav_points[i].icon[3],self._nav_points[i].icon[0])
			qp.drawText(self._nav_points[i].icon[0], self._nav_points[i].tag)
			# qp.setPen(self._color_edge)
			# qp.drawPolyline(self._nav_points[i].icon[0],self._nav_points[i].icon[1],
			# 	self._nav_points[i].icon[2],self._nav_points[i].icon[3],self._nav_points[i].icon[0])
			#print 'nav pt',i,' ',self._nav_points[i].icon[0]

	def update_nav_relative(self):
		for i in range(len(self._nav_points)):
			self._nav_points[i].get_distance(self._rover_nav.longitude,self._rover_nav.latitude)
			self._nav_points[i].get_heading(self._rover_nav.longitude,self._rover_nav.latitude)
			print 'nav',i,':',self._nav_points[i].longitude, ' ',self._nav_points[i].latitude,' ', self._nav_points[i].headingAngle,' ', self._nav_points[i].distance,self._nav_points[i].tag

	def Slot_Insert_Nav_Point(self, longitude,latitude,tag):
		self._nav_points.append(self.NavPoint(longitude,latitude, tag))
		self._nav_points[len(self._nav_points)-1].get_distance(self._rover_nav.longitude,self._rover_nav.latitude)
		self._nav_points[len(self._nav_points)-1].get_heading(self._rover_nav.longitude,self._rover_nav.latitude)
		print 'nav',len(self._nav_points)-1,':',self._nav_points[len(self._nav_points)-1].longitude, ' ',self._nav_points[len(self._nav_points)-1].latitude,' ', self._nav_points[len(self._nav_points)-1].headingAngle,' ', self._nav_points[len(self._nav_points)-1].distance, tag
		self.repaint()
	def Slot_Update_Rover_GPS(self, longitude,latitude):
		self._rover_nav.longitude = longitude
		self._rover_nav.latitude = latitude
		self.update_nav_relative()
		self.repaint()
	def Slot_Move_Camera_X(self, heading):
		if 0 <= heading and heading <= 180:
			self._camera_heading = heading
		elif 180 <= heading and heading <= 360:
			self._camera_heading = heading - 360
		print 'heading:',heading, self._camera_heading, '<', -180+self._fov_width
		for i in range(len(self._nav_points)): #update all nav point's positions on screen
			for j in range(len(self._nav_points[i].icon)):
				# in the case of numbers (180-fov<180<180+fov), numbers would break. Tread it as 180+x, instead of looping back to -180.
				# for a -179 pt, there would have to probably be something equivalent...
				if self._nav_points[i].headingAngle > 0 and self._camera_heading <= -180+self._fov_width:
					self._camera_heading = 360 + self._camera_heading
					self._nav_points[i].icon[j].setX(self._nav_point_Diamond[j].x() + 
						int((self._nav_points[i].headingAngle-self._camera_heading) * self._heading_translation_x))
				elif self._nav_points[i].headingAngle < 0 and self._camera_heading >= 180-self._fov_width:
					self._camera_heading = -360 + self._camera_heading
					self._nav_points[i].icon[j].setX(self._nav_point_Diamond[j].x() + 
						int((self._nav_points[i].headingAngle-self._camera_heading) * self._heading_translation_x))
				else:
					self._nav_points[i].icon[j].setX(self._nav_point_Diamond[j].x() + 
						int((self._nav_points[i].headingAngle-self._camera_heading) * self._heading_translation_x))
		
		self.repaint()
	def Slot_Move_Camera_Y(self, camera_up):
		self._camera_Up = camera_up
		for i in range(len(self._nav_points)): #update all nav point's positions on screen
			for j in range(len(self._nav_points[i].icon)):
				self._nav_points[i].icon[j].setY(self._nav_point_Diamond[j].y() + 
					int((self._nav_points[i].heightAngle+self._camera_Up) * self._heading_translation_y)) #addition because point should go down, not up when camera looks up
		self.repaint()

	# Used by Rover Tilt Info Widget
	class NavPoint():
		def __init__(self, longitude,latitude, tag):
			self.latitude = latitude
			self.longitude = longitude
			self.tag = tag
			self.distance = 0
			self.headingAngle = 0
			self.heightAngle = 0
			self.icon = []
			self.icon.append(QPoint(9,0))
			self.icon.append(QPoint(0,9))
			self.icon.append(QPoint(-9,0))
			self.icon.append(QPoint(0,-9))

		def get_distance(self, lon1,lat1):
			lon2 = self.longitude
			lat2 = self.latitude
			R = 6371 #idk why I use this; I think maybe the distance in pixels on the map???
			dLat = (lat2-lat1)*math.pi
			dLon = (lon2-lon2)*math.pi
			lat1 = lat1*math.pi
			lat2 = lat2*math.pi
			a = math.sin(dLat/2.0) * math.sin(dLat/2) + math.sin(dLon/2.0) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)
			c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
			d = R * c
			self.distance = d

		def get_heading(self, lon1,lat1):
			lon2 = self.longitude
			lat2 = self.latitude
			y = math.sin(lon2-lon1) * math.cos(lat2)
			x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2-lon1)
			heading = math.atan2(y,x)/math.pi * 180.0 #convert to degrees
			self.headingAngle = heading

class HelpWidget(QWidget):
	def __init__(self, width,height, parent=None):
		super(HelpWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.color_Edge = QColor(255,0,0)
		self.color_Background = QColor(255,0,0,200)
		self.color_Text = QColor(255,255,255)

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		self.drawHelp(qp)
		qp.end()

	def drawHelp(self, qp):
		qp.setPen(self.color_Edge)
		qp.setBrush(self.color_Background)
		qp.drawRect(0,0, self.width,self.height)

		qp.setPen(self.color_Text)
		qp.setBrush(Qt.NoBrush)
		qp.setFont(QFont('Decorative', 20))
		qp.drawText(0,0, "HELP Menu")
		qp.setFont(QFont('Decorative', 12))
		qp.drawText(0,1*20, "Shortcuts:")
		qp.drawText(10,2*20, "Q: show/hide Message Picture")
		qp.drawText(10,3*20, "Z: done current task")
		qp.drawText(10,4*20, "X: undo previous task")
		qp.drawText(10,5*20, "C: change task list display(1/2/3)")
		qp.drawText(10,6*20, "/: This help menu")

# Cleaned
class SignalQualityWidget(QWidget):
	Signal_Update_Link_Quality = pyqtSignal(str)
	Signal_Update_Signal_Strength = pyqtSignal(str)
	Signal_Update_Ping = pyqtSignal(str)
	def __init__(self, width,height, parent=None):
		super(SignalQualityWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self._color_Edge = QColor(255,0,0,200)
		self._color_Background = QColor(255,0,0,100)
		self._color_Text = QColor(255,255,255)
		self._draw_Link_Quality = "LQ:70/70"
		self._draw_Signal_Strength = "SS-58"
		self._draw_Ping = "P1.23"
		self.Signal_Update_Link_Quality.connect(self._Slot_Update_Link_Quality)
		self.Signal_Update_Signal_Strength.connect(self._Slot_Update_Signal_Strength)
		self.Signal_Update_Ping.connect(self._Slot_Update_Ping)

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		self.drawText(qp)
		qp.end()

	def drawText(self, qp):
		qp.setPen(self._color_Edge)
		qp.setBrush(self._color_Background)
		qp.drawRect(0,0, self.width,self.height)

		qp.setPen(self._color_Text)
		qp.setFont(QFont('Decorative', 12))
		qp.drawText(0,20, self._draw_Link_Quality)
		qp.drawText(0+12*7,20, self._draw_Signal_Strength)
		qp.drawText(0+12*7+12*5, 20, self._draw_Ping)

	def _Slot_Update_Link_Quality(self, linkQuality):
		self._draw_Link_Quality = "LQ:"+linkQuality
		self.repaint()
	def _Slot_Update_Signal_Strength(self, sigStr):
		self._draw_Signal_Strength = "SS:"+sigStr
		self.repaint()
	def _Slot_Update_Ping(self, ping):
		self._draw_Ping = "P"+ping
		self.repaint()

class MinimapWidget(QWidget):
	Signal_Set_Size = pyqtSignal(int,int)
	Signal_Clear_Nav_Point = pyqtSignal()
	Signal_Nav_Point = pyqtSignal(float,float, QString)
	Signal_Clear_Nav_Route = pyqtSignal()
	Signal_Nav_Route = pyqtSignal(float,float, float,float)
	Signal_View_GPS = pyqtSignal(float,float)
	Signal_Update_Rover_GPS = pyqtSignal(float,float)
	Signal_Update_Camera_Heading_X = pyqtSignal(float)
	Signal_Update_Rover_Heading = pyqtSignal(float)
	Signal_Update_Map_Type = pyqtSignal(int)#0=Topo 1=Aerial
	#Signal_Increase_Zoom = pyqtSignal(float)
	def __init__(self, width, height, parent = None):
		super(MinimapWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Message</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		
		self.message_text = "OperatorGUI"
		#self.img = QImage('../resources/q3128_DRG24k-c.tiff')
		self._topoQuads = []# img,W,E,S,N,x,y
		self._topoQuads.append([QImage(path+'/resources/q3128_DRG24k-c.tiff'), -110.875000, -110.750000, 38.375000, 38.500000,4485,5697])
		# self._topoQuads.append([QImage('q3129_DRG24k-c.tiff'), -110.750000, -110.625000, 38.375000, 38.500000,4493,5703])
		# self._topoQuads.append([QImage('q3228_DRG24k-c.tiff'), -110.875000, -110.750000, 38.250000, 38.375000,4493,5703])
		# self._topoQuads.append([QImage('q3229_DRG24k-c.tiff'), -110.750000, -110.625000, 38.250000, 38.375000,4501,5703])
		self._AerialQuads = []
		self._AerialQuads.append([QImage(path+'/resources/q3128_NAIP2011_4-band.jpg'), -110.875000, -110.750000, 38.375000, 38.500000,11570,14520])

		self._nav_points = []
		self.route_points = []
		self._PIXEL_COORD = []
		for i in range(len(self._topoQuads)):
			self._PIXEL_COORD.append([0.125000/self._topoQuads[i][5],0.125000/self._topoQuads[i][6]])
		self._PIXEL_AERIAL_COORD = []
		for i in range(len(self._AerialQuads)):
			self._PIXEL_AERIAL_COORD.append([0.125000/self._AerialQuads[i][5],0.125000/self._AerialQuads[i][6]])
		# print '_PIXEL_COORD:',self._PIXEL_COORD[0][0],':',self._PIXEL_COORD[0][1]
		# print '_PIXEL_AERIAL_COORD:',self._PIXEL_AERIAL_COORD[0][0],':',self._PIXEL_AERIAL_COORD[0][1]

		self._PIXEL_X = 0.00002787068004 #0.125000/4485
		self._PIXEL_Y = 0.00002194137265 #0.125000/5697
		self._longitude= -110.875000 + 0.0625#y; init as middle of map
		self._latitude =   38.375000 + 0.0625#x; init as middle of map
		self._longitude_x = (self._longitude - -110.875000) / self._PIXEL_X# self.width/2
		self._latitude_y  = (self._latitude  -   38.375000) / self._PIXEL_Y# self.height/2
		self._camera_heading = 0
		self._camera_fov = 60
		self._camera_fov_length = 20
		self._rover_longitude = self._longitude
		self._rover_latitude = self._latitude
		self._rover_heading = 0
		self.Signal_Set_Size.connect(self.Slot_Set_Size)
		self.Signal_Clear_Nav_Point.connect(self.Slot_Clear_Nav_Point)
		self.Signal_Nav_Point.connect(self.Slot_Nav_Point)
		self.Signal_Clear_Nav_Route.connect(self.Slot_Clear_Nav_Route)
		self.Signal_Nav_Route.connect(self.Slot_Nav_Route)
		self.Signal_View_GPS.connect(self.Slot_View_GPS)
		self.Signal_Update_Rover_GPS.connect(self.Slot_Update_Rover_GPS)
		self.Signal_Update_Camera_Heading_X.connect(self.Slot_Update_Camera_Heading_X)
		self.Signal_Update_Rover_Heading.connect(self.Slot_Update_Rover_Heading)
		#self.Signal_Increase_Zoom.connect(self.Slot_Increase_Zoom)
		self.Signal_Update_Map_Type.connect(self.Slot_Update_Map_Type)

		self.roverHeading = MinimapWidget.RoverHeadingWidget(20,20, self)
		self.roverHeading.move(self.width/2-self.roverHeading.width/2,self.height/2-self.roverHeading.height/2)
		self.roverFOV = MinimapWidget.RoverFOVWidget(20,20, self)
		self.roverFOV.move(self.width/2-self.roverFOV.width/2,self.height/2-self.roverFOV.height/2)
		self._pixmap = QPixmap(self.width,self.height)
		self._zoom = 1.0
		self._Map_Types = 0

	def decimal_to_pixel(self, decimal, division):
		result = (decimal) / (division*self._zoom)# self.width/2
		return result
	def long_to_pixel_x(self, longitude):
		result = (longitude) / (self._PIXEL_X*self._zoom)# self.width/2
		return result
	def lati_to_pixel_y(self, latitude):
		result = ((latitude) / (self._PIXEL_Y*self._zoom))# self.height/2
		return result

	def paintEvent(self, event):
		qp = QPainter()
		self._pixmap = QPixmap(self.width,self.height)
		qp.begin(self._pixmap)
		qp.translate(self.width/2, self.height/2)
		self.drawMap(event, qp)
		self.drawNavPoints(event, qp)
		self.drawNavRoutes(event, qp)
		self.drawGPSInfo(event, qp)
		qp.end()
		qp.begin(self)
		qp.setOpacity(0.5)
		qp.setPen(QColor(255, 34, 3))
		qp.setBrush(QColor(255, 34, 3,50))
		qp.drawRect(0,0, self.width,self.height)
		qp.drawPixmap(0,0, self.width,self.height, self._pixmap)
		qp.drawText(10,10, str(self._zoom))
		qp.setOpacity(1)
		qp.end()
		

	def drawMap(self, event, qp):
		# print 'orig:',self._longitude_x-windowWidth/2,':',self._latitude_y-windowHeight/2,' ',windowWidth,':',windowHeight
		# print 'wonder',self._latitude,'+',(windowHeight/2)*self._PIXEL_Y,'=',self._latitude + (windowHeight/2)*self._PIXEL_Y

		if self._Map_Types == 0:
			windowWidth = (self.width)*self._zoom
			windowHeight = (self.height)*self._zoom
			# self._longitude_x = (self._longitude - self._topoQuads[0][1]) / self._PIXEL_X# self.width/2
			# self._latitude_y = 5697-((self._latitude   -   self._topoQuads[0][3]) / self._PIXEL_Y)# self.height/2
			img_rect = QRect(self._longitude_x-windowWidth/2,self._latitude_y-windowHeight/2, windowWidth,windowHeight)
			posRect = QRect(-self.width/2, -self.height/2, self.width,self.height)
			qp.drawImage(posRect, self._topoQuads[0][0], img_rect)


		#self._PIXEL_AERIAL_COORD
		if self._Map_Types == 1:
			windowWidth = (self.width*(11570.0/4485.0))*self._zoom
			windowHeight = (self.height*(14520/5697))*self._zoom
			aerialLongitudeX = (self._longitude - self._AerialQuads[0][1]) / self._PIXEL_AERIAL_COORD[0][0]# self.width/2
			aerialLatitudeY  = self._AerialQuads[0][6]-((self._latitude   -   self._AerialQuads[0][3]) / self._PIXEL_AERIAL_COORD[0][1])# self.height/2
			img_rect = QRect(aerialLongitudeX-windowWidth/2,aerialLatitudeY-windowHeight/2, windowWidth,windowHeight)
			posRect = QRect(-self.width/2, -self.height/2, self.width,self.height)
			qp.drawImage(posRect, self._AerialQuads[0][0], img_rect)

		# # Coordinates only
		# longitudeCoordExtend = windowWidth*self._PIXEL_X
		# latitudeCoordExtend = windowHeight*self._PIXEL_Y

		# # [longitude,latitude,quadrant types]
		# cornerCoords = []
		# cornerCoords.append([self._longitude - longitudeCoordExtend/2.0, self._latitude  + latitudeCoordExtend/2.0]) #top left corner
		# cornerCoords.append([self._longitude + longitudeCoordExtend/2.0, self._latitude  + latitudeCoordExtend/2.0]) #top right corner
		# cornerCoords.append([self._longitude - longitudeCoordExtend/2.0, self._latitude  - latitudeCoordExtend/2.0]) #bottom left corner
		# cornerCoords.append([self._longitude + longitudeCoordExtend/2.0, self._latitude  - latitudeCoordExtend/2.0]) #bottom right corner
		# for i in range(len(cornerCoords)):
		# 	cornerCoords[i].append(self.coordinateInQuadrant(cornerCoords[i][0],cornerCoords[i][1]))
		# 	print 'coords',i,':',cornerCoords[i][0],' ',cornerCoords[i][1],cornerCoords[i][2]

		# # get midpoint coordinates if they exist
		# hasLongitudeBorder = False
		# hasLatitudeBorder = False
		# longitudeCoordMid = -1
		# latitudeCoordMid = -1
		# #if cornerCoords[0][2] is not None and cornerCoords[1][2] is not None:
		# if cornerCoords[0][2] is not cornerCoords[1][2]:
		# 	if cornerCoords[0][2] is not None:
		# 		longitudeCoordMid = self._topoQuads[cornerCoords[0][2]][2] # get width for top Left Quad
		# 	elif cornerCoords[1][2] is not None:
		# 		longitudeCoordMid = self._topoQuads[cornerCoords[1][2]][2] # get width for top Left Quad
		# 	hasLongitudeBorder = True
		# #if cornerCoords[0][2] is not None and cornerCoords[2][2] is not None:
		# if cornerCoords[0][2] is not cornerCoords[2][2]:
		# 	if cornerCoords[0][2] is not None:
		# 		latitudeCoordMid = self._topoQuads[cornerCoords[0][2]][3] # get height for top Left Quad
		# 	elif cornerCoords[2][2] is not None:
		# 		latitudeCoordMid = self._topoQuads[cornerCoords[2][2]][3] # get height for top Left Quad
		# 	hasLatitudeBorder = True

		# # coordinates of the top left corners for each map to display
		# quadCoords = []
		# quadCoords.append([cornerCoords[0][0],cornerCoords[0][1]])
		# if hasLongitudeBorder:
		# 	quadCoords.append([longitudeCoordMid,cornerCoords[0][1]])#top right quadrant
		# else:
		# 	quadCoords.append(-1)
		# if hasLatitudeBorder:
		# 	quadCoords.append([cornerCoords[0][0],latitudeCoordMid])#bottom left quadrant
		# else:
		# 	quadCoords.append(-1)
		# if hasLongitudeBorder and hasLatitudeBorder:
		# 	quadCoords.append([longitudeCoordMid,latitudeCoordMid])#bottom left quadrant
		# else:
		# 	quadCoords.append(-1)
		# for i in range(len(quadCoords)):
		# 	print 'quadCoords',i,':',quadCoords[i]

		# coordXLength = []
		# if hasLongitudeBorder:#set top left quadrant's width (in coordinates)
		# 	coordXLength.append(cornerCoords[0][0] - longitudeCoordMid)
		# else:
		# 	coordXLength.append(cornerCoords[0][0] - cornerCoords[1][0])
		# if hasLongitudeBorder:#set top right quadrant's width (in coordinates)
		# 	coordXLength.append(longitudeCoordMid - cornerCoords[1][0])
		# else:
		# 	coordXLength.append(-1)
		# if hasLatitudeBorder:#set bottom left quadrant's width (in coordinates)
		# 	coordXLength.append(coordXLength[0])#same as top left quadrant's width
		# else:
		# 	coordXLength.append(-1)
		# if hasLongitudeBorder and hasLatitudeBorder:
		# 	coordXLength.append(longitudeCoordMid - cornerCoords[1][0])#same as top left quadrant's width
		# else:
		# 	coordXLength.append(-1)
		# for i in range(len(coordXLength)):
		# 	print 'coordXLength',i,':',coordXLength[i]

		# coordYLength = []
		# if hasLatitudeBorder:# top left quadrant's height
		# 	coordYLength.append(cornerCoords[0][1] - latitudeCoordMid)
		# else:
		# 	coordYLength.append(cornerCoords[0][1] - cornerCoords[2][1])
		# if hasLongitudeBorder:# top right = top left
		# 	coordYLength.append(coordYLength[0])
		# else:
		# 	coordYLength.append(-1)
		# if hasLatitudeBorder:# bottom left
		# 	coordYLength.append(latitudeCoordMid - cornerCoords[2][1])
		# else:
		# 	coordYLength.append(-1)
		# if hasLongitudeBorder and hasLatitudeBorder:# bottom right = bottom left
		# 	coordYLength.append(coordYLength[2])
		# else:
		# 	coordYLength.append(-1)
		# #for i in range(len(coordYLength)):
		# 	print 'coordYLength',i,':',coordYLength[i]

		# # turn quadrant coordinates to pixels relative to image(top left corner = 0,0)
		# quadPixels = []
		# for i in range(len(quadCoords)):
		# 	if cornerCoords[i][2] is not None:
		# 		if quadCoords[i] is not -1:
		# 			longPixel = self.long_to_pixel_x(quadCoords[i][0] - self._topoQuads[cornerCoords[i][2]][1])
		# 			latiPixel = self.lati_to_pixel_y(quadCoords[i][1] - self._topoQuads[cornerCoords[i][2]][3])
		# 			quadPixels.append([longPixel,latiPixel])
		# 			print 'quadPixels',i,'=',self._topoQuads[cornerCoords[i][2]][1],':',quadCoords[i][0],' ',self._topoQuads[cornerCoords[i][2]][3],':',quadCoords[i][1]
		# 		else:
		# 			quadPixels.append(-1)
		# 			print 'quadPixels',i,':',-1
		# 	else:
		# 			quadPixels.append(-1)
		# 			print 'quadPixels',i,':',-1
		# # turn quadrant sizes to pixel lengths
		# sizePixels = []
		# for i in range(len(coordXLength)):
		# 	longPixel = self.long_to_pixel_x(-1.0*coordXLength[i])
		# 	latiPixel = self.lati_to_pixel_y(coordYLength[i])
		# 	sizePixels.append([longPixel,latiPixel])
		# 	print 'sizePixels',i,':',sizePixels[i]

		# #topleft image
		# i = 0
		# if cornerCoords[i][2] is not None:
		# 	print 'print 1'
		# 	img_rect = QRect(quadPixels[i][0],self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 	posRect = QRect(-self.width/2,-self.height/2, sizePixels[i][0],sizePixels[i][1])
		# 	qp.drawImage(posRect, self._topoQuads[cornerCoords[i][2]][0], img_rect)
		# 	print 'img_rect:',quadPixels[i][0],':',self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# 	print 'posRect',-self.width/2,':',-self.height/2,' ',sizePixels[i][0],':',sizePixels[i][1]
		# if hasLongitudeBorder:
		# 	i = 1
		# 	if cornerCoords[i][2] is not None:
		# 		print 'print 2'
		# 		img_rect = QRect(quadPixels[i][0],self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 		posRect = QRect(self.width/2-sizePixels[i][0],-self.height/2, sizePixels[i][0],sizePixels[i][1])
		# 		qp.drawImage(posRect, self._topoQuads[cornerCoords[i][2]][0], img_rect)
		# 		print 'img_rect:',quadPixels[i][0],':',self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# 		print 'posRect:',self.width/2-sizePixels[i][0],':',-self.height/2,' ',sizePixels[i][0],':',sizePixels[i][1]
		# if hasLatitudeBorder:
		# 	i = 2
		# 	if cornerCoords[i][2] is not None:
		# 		print 'print 3'
		# 		img_rect = QRect(quadPixels[i][0],self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 		posRect = QRect(-self.width/2,-self.height/2+sizePixels[i-2][1], sizePixels[i][0],sizePixels[i][1])
		# 		qp.drawImage(posRect, self._topoQuads[cornerCoords[i][2]][0], img_rect)
		# 		# -self.width/2,-self.height/2+sizePixels[i-2][1]
		# 		print 'img_rect:',quadPixels[i][0],':',self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# 		print 'posRect:',-self.width/2,':',-self.height/2+sizePixels[i-2][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# if hasLongitudeBorder and hasLatitudeBorder:
		# 	i = 3
		# 	if cornerCoords[i][2] is not None:
		# 		print 'print 4'
		# 		img_rect = QRect(quadPixels[i][0],self._topoQuads[cornerCoords[i][2]][6]-quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 		posRect = QRect(self.width/2-sizePixels[i][0],-self.height/2+sizePixels[i-2][1], sizePixels[i][0],sizePixels[i][1])
		# 		qp.drawImage(posRect, self._topoQuads[cornerCoords[i][2]][0], img_rect)
		# 		print 'img_rect:',quadPixels[i][0],':',quadPixels[i][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# 		print 'posRect:',self.width/2-sizePixels[i][0],':',-self.height/2+sizePixels[i-2][1],' ',sizePixels[i][0],':',sizePixels[i][1]
		# print ''


		#topright image
		# if hasLongitudeBorder:
		# 	i = 1
		# 	img_rect = QRect(quadPixels[i][0],quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 	posRect = QRect(-self.width/2, -self.height/2,sizePixels[i][0],sizePixels[i][1])
		# 	qp.drawImage(posRect, self._topoQuads[0][0], img_rect)
		# if hasLatitudeBorder:
		# 	i = 2
		# 	img_rect = QRect(quadPixels[i][0],quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 	posRect = QRect(-self.width/2, -self.height/2,sizePixels[i][0],sizePixels[i][1])
		# 	qp.drawImage(posRect, self._topoQuads[0][0], img_rect)
		# if hasLongitudeBorder and hasLatitudeBorder:
		# 	i = 3
		# 	img_rect = QRect(quadPixels[i][0],quadPixels[i][1],sizePixels[i][0],sizePixels[i][1])
		# 	posRect = QRect(-self.width/2, -self.height/2,sizePixels[i][0],sizePixels[i][1])
		# 	qp.drawImage(posRect, self._topoQuads[0][0], img_rect)

		# img_rect = QRect(self._longitude_x-windowWidth/2,self._latitude_y-windowHeight/2, windowWidth,windowHeight)
		# posRect = QRect(-self.width/2, -self.height/2, self.width,self.height)
		# qp.drawImage(posRect, self._topoQuads[0][0], img_rect)


		# #self._topoQuads = []# img,W,E,S,N
		# # convert corners to pixels
		# cornerPixels = []
		# cornerPixels.append([(cornerCoords[0][0] - self._topoQuads[cornerCoords[0][2]][1]) / self._PIXEL_X
		#                    ,(cornerCoords[0][1] - self._topoQuads[cornerCoords[0][2]][4]) / self._PIXEL_Y])
		# cornerPixels.append([(cornerCoords[1][0] - self._topoQuads[cornerCoords[1][2]][2]) / self._PIXEL_X
		#                    ,(cornerCoords[1][1] - self._topoQuads[cornerCoords[1][2]][4]) / self._PIXEL_Y])
		# cornerPixels.append([(cornerCoords[2][0] - self._topoQuads[cornerCoords[2][2]][1]) / self._PIXEL_X
		#                    ,(cornerCoords[2][1] - self._topoQuads[cornerCoords[2][2]][3]) / self._PIXEL_Y])
		# cornerPixels.append([(cornerCoords[3][0] - self._topoQuads[cornerCoords[3][2]][2]) / self._PIXEL_X
		#                    ,(cornerCoords[3][1] - self._topoQuads[cornerCoords[3][2]][3]) / self._PIXEL_Y])

		# # get length and width of quadrant 1 to show
		# quadSizes = []
		# quadSizes.append((longitudeX - cornerCoords[0][0]) / self._PIXEL_X)
		# quadSizes.append((latitudeY  - cornerCoords[0][1]) / self._PIXEL_Y)
		# if hasLongitudeBorder:
		# 	quadSizes.append((cornerCoords[1][0] - longitudeX) / self._PIXEL_X)#midlines - right corner
		# 	quadSizes.append((cornerCoords[1][1] -  latitudeY) / self._PIXEL_Y)#
		# # bottom left
		# if hasLatitudeBorder:
		# 	print 'blehbleh'



		# #TOPLEFT CORNER # img,W,E,S,N
		# cornerX1 = (self._longitude-longitudeCoordExtend - self._topoQuads[0][1]) / self._PIXEL_X
		# cornerY1 = (self._latitude+latitudeCoordExtend   - self._topoQuads[0][3]) / self._PIXEL_Y
		# quadWidth = (longitudeX - self._topoQuads[0][1]) / self._PIXEL_X
		# quadHeight= (latitudeY  - self._topoQuads[0][3]) / self._PIXEL_Y

		# img_rect = QRect(cornerX1,cornerY1, quadWidth,quadHeight)
		# posRect = QRect(-self.width/2, -self.height/2, self.width,self.height)
		# qp.drawImage(posRect, self._topoQuads[0][0], img_rect)

	def coordinateInQuadrant(self, longitude,latitude):
		for i in range(len(self._topoQuads)):# img,W,E,S,N
			if self._topoQuads[i][1] <= longitude <= self._topoQuads[i][2] and self._topoQuads[i][3] <= latitude <= self._topoQuads[i][4]:
				return i
		return None

	def drawNavPoints(self, event,qp):
		qp.setPen(QColor(255, 34, 3))
		qp.setBrush(QColor(255, 34, 3,50))
		qp.setFont(QFont('times',8))
		diviX = -1
		if self._Map_Types == 0:
			diviX = self._PIXEL_COORD[0][0]
		elif self._Map_Types == 1:
			diviX = self._PIXEL_AERIAL_COORD[0][0]
		diviY = -1
		if self._Map_Types == 0:
			diviY = self._PIXEL_COORD[0][1]
		elif self._Map_Types == 1:
			diviY = self._PIXEL_AERIAL_COORD[0][1]

		for i in range(len(self._nav_points)):
			symbol_size = 4
			long_screen = self._nav_points[i][0] - self._longitude
			lati_screen = self._latitude - self._nav_points[i][1]
			pixel_x = self.decimal_to_pixel(long_screen,self._PIXEL_COORD[0][0])
			pixel_y = self.decimal_to_pixel(lati_screen,self._PIXEL_COORD[0][1])
			qp.drawEllipse(pixel_x-symbol_size/2,pixel_y-symbol_size/2, symbol_size,symbol_size)
			qp.drawText(pixel_x,pixel_y,self._nav_points[i][2])
			#print 'pt',i,' ',long_screen,' ',lati_screen,' ',pixel_x,' ',pixel_y

	def drawNavRoutes(self, event,qp):
		qp.setPen(QColor(255, 34, 3))
		qp.setBrush(QColor(255, 34, 3,50))
		diviX = -1
		if self._Map_Types == 0:
			diviX = self._PIXEL_COORD[0][0]
		elif self._Map_Types == 1:
			diviX = self._PIXEL_AERIAL_COORD[0][0]
		diviY = -1
		if self._Map_Types == 0:
			diviY = self._PIXEL_COORD[0][1]
		elif self._Map_Types == 1:
			diviY = self._PIXEL_AERIAL_COORD[0][1]

		for i in range(len(self.route_points)):
			long_screen = []
			lati_screen = []
			pixel_x = []
			pixel_y = []
			for j in range(2):
				long_screen.append(self._longitude - self.route_points[i][j][0])
				lati_screen.append( self._latitude - self.route_points[i][j][1])
				pixel_x.append(self.decimal_to_pixel(long_screen[j],self._PIXEL_COORD[0][0]))
				pixel_y.append(self.decimal_to_pixel(lati_screen[j],self._PIXEL_COORD[0][1]))
			qp.drawLine(-pixel_x[0],pixel_y[0], -pixel_x[1],pixel_y[1])
			#-pixel_x,pixel_y

		#self.route_points

	def drawGPSInfo(self, event, qp):
		qp.setPen(QColor(255, 34, 3))
		qp.setBrush(QColor(255, 34, 3))
		qp.setFont(QFont('Decorative', 14))
		qp.drawText(self.width/2-20*15, self.height/2-20, str(self._rover_longitude)+":"+str(self._rover_latitude))

		qp.setPen(QColor(168, 34, 3))
		qp.setBrush(Qt.NoBrush)
		qp.drawRect(-self.width/2, -self.height/2, self.width-1, self.height-1)#border

	def repaintRover(self):
		long_screen = self._longitude - self._rover_longitude
		lati_screen = self._latitude - self._rover_latitude
		pixel_x = self.decimal_to_pixel(long_screen,self._PIXEL_COORD[0][0])
		pixel_y = self.decimal_to_pixel(lati_screen,self._PIXEL_COORD[0][1])
		self.roverHeading.move(self.width/2+-1*pixel_x-self.roverHeading.width/2,self.height/2+pixel_y-self.roverHeading.height/2)
		self.roverFOV.move(self.width/2+-1*pixel_x-self.roverFOV.width/2,self.height/2+pixel_y-self.roverFOV.height/2)
		
	def Slot_Set_Size(self,width,height):
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.repaint()
	def Slot_Clear_Nav_Point(self):
		self._nav_points = []
	def Slot_Nav_Point(self, longitude,latitude, name):
		self._nav_points.append([longitude,latitude,name])
		self.repaint()
	def Slot_Clear_Nav_Route(self):
		self.route_points = []
	def Slot_Nav_Route(self, longitude,latitude, longitude2,latitude2):
		self.route_points.append([[longitude,latitude], [longitude2,latitude2]])
		self.repaint()
	def Slot_View_GPS(self, longitude,latitude):
		self._latitude = latitude
		self._longitude = longitude
		self._longitude_x = (self._longitude - self._topoQuads[0][1]) / self._PIXEL_X# self.width/2
		self._latitude_y = self._topoQuads[0][6]-((self._latitude   -   self._topoQuads[0][3]) / self._PIXEL_Y)# self.height/2
		#self._latitude_y = 5697-((self._latitude   -   self._topoQuads[3]) / self._PIXEL_Y)# self.height/2
		#print self._longitude,' ',self._latitude
		self.repaintRover()
		self.repaint()
	def Slot_Update_Rover_GPS(self, longitude,latitude):
		self._rover_longitude = longitude
		self._rover_latitude = latitude
		self.repaintRover()
		self.repaint()
	def Slot_Update_Camera_Heading_X(self, heading):
		self._camera_heading = heading
		self.roverFOV.Signal_Heading.emit(self._camera_heading)
		self.repaint()
	def Slot_Update_Rover_Heading(self, heading):
		self._rover_heading = heading
		self.roverHeading.Signal_Heading.emit(self._rover_heading)
	def Slot_Update_Map_Type(self, num):
		self._Map_Types = num
		self.repaint()
	def zoomIncrease(self, zoom):
		self._zoom += zoom
		self.repaintRover()
		self.repaint()
	def zoomDecrease(self, zoom):
		if self._zoom > 0.2:
			self._zoom -= zoom
			self.repaintRover()
			self.repaint()
	def zoomSet(self, zoom):
		self._zoom = zoom
		self.repaintRover()
		self.repaint()
	def popNavPoint(self):
		if len(self._nav_points) > 0:
			return self._nav_points.pop()
	def popNavRoute(self):
		if len(self.route_points) > 0:
			return self.route_points.pop()

	class RoverFOVWidget(QWidget):
		Signal_Heading = pyqtSignal(float)
		def __init__(self, width,height, parent = None):
			super(MinimapWidget.RoverFOVWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.resize(self.width,self.height)
			self._camera_fov = self.parent._camera_fov
			self._camera_fov_length = self.parent._camera_fov_length
			self._camera_heading = self.parent._camera_heading
			self.Signal_Heading.connect(self.Slot_Heading)

		def paintEvent(self, event):
			qp = QPainter()
			qp.begin(self)
			qp.translate(self.width/2, self.height/2)
			qp.rotate(self._camera_heading)
			self.draw_rover_heading(event, qp)
			qp.end()

		def draw_rover_heading(self, event, qp):
			qp.setPen(QColor(168, 34, 3))
			qp.setBrush(QColor(168, 34, 3))
			qp.setFont(QFont('Decorative', 20))

			self.roverCircle = 5
			qp.drawEllipse(-self.roverCircle/2,-self.roverCircle/2, self.roverCircle,self.roverCircle) # draw rover dot
			endX = self._camera_fov_length*math.cos((self._camera_fov/180.0)*math.pi)
			endY = self._camera_fov_length*math.sin((self._camera_fov/180.0)*math.pi)
			qp.drawLine(0,0, endX,-endY) #left line
			qp.drawLine(0,0, -endX,-endY) #right line

		def Slot_Heading(self, heading):
			self._camera_heading = heading
			#print str(self._camera_heading)
			self.repaint()

	class RoverHeadingWidget(QWidget):
		Signal_Heading = pyqtSignal(float)
		def __init__(self, width,height, parent = None):
			super(MinimapWidget.RoverHeadingWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.resize(self.width,self.height)
			self._camera_fov = self.parent._camera_fov
			self._camera_fov_length = self.parent._camera_fov_length
			self._rover_heading = self.parent._rover_heading
			self.Signal_Heading.connect(self.Slot_Heading)

		def paintEvent(self, event):
			qp = QPainter()
			qp.begin(self)
			qp.translate(self.width/2, self.height/2)
			qp.rotate(self._rover_heading)
			self.draw_rover_heading(event, qp)
			qp.end()

		def draw_rover_heading(self, event, qp):
			qp.setPen(QColor(255,255,0))
			qp.setBrush(QColor(255,255, 0))
			qp.setFont(QFont('Decorative', 20))

			self.pointerLength = 10
			self.roverCircle = 5/2
			pt1 = QPoint(-self.roverCircle,-self.roverCircle)
			pt2 = QPoint(self.roverCircle,-self.roverCircle)
			pt3 = QPoint(0,-self.pointerLength)
			qp.drawPolyline(pt1,pt2,pt3,pt1)

		def Slot_Heading(self, heading):
			self._rover_heading = heading
			self.repaint()

class GraphWidget(QWidget):
	Signal_Add_Point = pyqtSignal(float)
	Signal_Remove_Point = pyqtSignal()
	def __init__(self, graphMax, width,height, parent=None):
		super(GraphWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self._color_Edge = QColor(255,0,0,200)
		self._color_Background = QColor(255,0,0,100)
		self._color_Lines = QColor(255,255,255)
		self.Signal_Add_Point.connect(self.Slot_Add_Point)
		self.Signal_Remove_Point.connect(self.Slot_Remove_Point)
		self._margin = 5
		self._graph_height = self.height-self._margin
		self._graph_gap_width = 20
		self._graph_Max = graphMax
		self._graph_points = []

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		self.drawBackground(qp)
		self.drawGraph(qp)
		qp.end()

	def drawGraph(self, qp):
		qp.setPen(self._color_Edge)
		for i in range(len(self._graph_points)-1):
			qp.drawLine(self._margin+(i*self._graph_gap_width),self.height-(self._graph_points[i]/self._graph_Max)*self._graph_height, 
				self._margin+(i+1)*self._graph_gap_width,self.height-(self._graph_points[i+1]/self._graph_Max)*self._graph_height)

	def drawBackground(self, qp):
		qp.setPen(self._color_Edge)
		qp.setBrush(self._color_Background)
		qp.drawRect(0,0, self.width,self.height)

	def Slot_Add_Point(self, point):
		self._graph_points.append(point)
		self.repaint()

	def Slot_Remove_Point(self):
		self._graph_points.remove(self._graph_points[0])
		self.repaint()

#INCOMPLETE
class CameraStatusWidget(QWidget):
	Signal_Camera_Status = pyqtSignal(int) #0=stop 1=pause 2:ready 3:playing
	Signal_Camera_Set = pyqtSignal(int)
	def __init__(self, width,height, videoSet, videoWidget, parent = None):
		super(CameraStatusWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.videoSet = videoSet
		self.videoWidget = videoWidget
		self.currentVideo = 0 #current video to show
		self.currentVideoStatus = 0 #current video status (stop,pause,start)
		self.Signal_Camera_Status.connect(self.Slot_Camera_Status)
		self.Signal_Camera_Set.connect(self.Slot_Camera_Set)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawWidget(event, qp)
		qp.end()

	def drawWidget(self, event, qp):
		qp.setPen(QColor(255,0,0))
		qp.setBrush(QColor(255,0,0, 100))
		qp.setFont(QFont('Decorative', 10))
		qp.drawRect(0,0, self.width-1,self.height-1)
		if self.currentVideoStatus == 0:# stop
			qp.drawRect(5,5, 20,20)
		elif self.currentVideoStatus == 1:#=pause
			qp.drawRect(5,5, 5,20)
			qp.drawRect(15,5, 5,20)
		elif self.currentVideoStatus == 2:#=ready
			qp.drawEllipse(5,5, 20,20)
		elif self.currentVideoStatus == 3:#=playing
			pt1 = QPoint(5,5)
			pt2 = QPoint(5,25)
			pt3 = QPoint(25,15)
			qp.drawPolygon(pt1,pt2,pt3)
		qp.drawText(30,20, self.videoSet[self.currentVideo][2])

	def Slot_Camera_Status(self, status):
		self.currentVideoStatus = status
		if self.currentVideoStatus == 0:#0=stop 1=pause 2=ready 3=playing
			self.videoSet[self.currentVideo][0].set_state(gst.STATE_NULL)
		elif self.currentVideoStatus == 1:
			self.videoSet[self.currentVideo][0].set_state(gst.STATE_PAUSED)
		elif self.currentVideoStatus == 2:
			self.videoSet[self.currentVideo][0].set_state(gst.STATE_READY)
		elif self.currentVideoStatus == 3:
			self.videoSet[self.currentVideo][0].set_state(gst.STATE_PLAYING)
		self.repaint()
	def Slot_Camera_Set(self, index):
		self.Slot_Camera_Status(0) #stop current video
		self.currentVideo = index
		self.videoWidget.Signal_Camera_Set.emit(self.currentVideo)
		self.repaint()

class VideoDisplayWidget(QWidget):
	Signal_Camera_Set = pyqtSignal(int)
	def __init__(self, width, height,videoSet, parent = None):
		super(VideoDisplayWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self._cameraIndex = 0
		self.videoSet = videoSet
		self._pixmaps = []
		for i in range(len(self.videoSet)):
			self.videoSet[i][1]._Signal_Buffer.signal.connect(self.Slot_Trigger_Do_Render)
			self._pixmaps.append(QPixmap())
		self.Signal_Camera_Set.connect(self.Slot_Camera_Set)

	def paintEvent(self, event):
		painter = QPainter()
		painter.begin(self)
		painter.drawPixmap(0,0, self.width,self.height, self._pixmaps[self._cameraIndex])
		painter.end()

	def Slot_Trigger_Do_Render(self, get_buffer, index):
		data = get_buffer.data
		self._pixmaps[index].loadFromData(data)
		self.repaint()

	def Slot_Camera_Set(self, index):
		self._cameraIndex = index

class SignalObject(QObject):
	signal = pyqtSignal(gst.Buffer,int)
	def __init__(self, parent = None):
		super(SignalObject, self).__init__(parent)

class VideoSink(gst.BaseSink):
	__gtype_name__ = 'VideoSink'
	__gstdetails__ = ("Video Sink", "Sink/Network",
	"Receive data over a CCNx network", "Derek Kulinski <takeda@takeda.tk>")

	__gsttemplates__ = (
		gst.PadTemplate("sink",
			gst.PAD_SINK,
			gst.PAD_ALWAYS,
			gst.caps_new_any()),
		)
	_index = 0
	_Signal_Buffer = SignalObject()

	def do_start(self):
		#print "Starting!"
		self.of = open("video.bin", "wb")
		return True

	def do_stop(self):
		#print "Stopping!"
		self.of.close()
		return True

	def do_set_caps(self, caps):
		#print "Caps: %s" % caps
		self.of.write("%s\n" % caps)
		return True

	def do_render(self, get_buffer): #gst.Buffer
		#print "Buffer timestamp %d %d %d" % (get_buffer.timestamp, get_buffer.duration, get_buffer.flags)
		self._Signal_Buffer.signal.emit(get_buffer, self._index)
		return gst.FLOW_OK

	def do_preroll(self, buf):
		#print "Preroll"
		return gst.FLOW_OK

	def do_event(self, ev):
		#print "Got event of type %s" % ev.type
		return gst.FLOW_OK