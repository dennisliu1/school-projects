import math
import sys
from PyQt4.QtGui import QMainWindow,QFont,QToolTip,QTabWidget,QLabel,QWidget,QLCDNumber,QIcon,QDesktopWidget,QColor,QImage,QPixmap,QPainter
from PyQt4.QtCore import QThread,Qt,pyqtSignal,QString,QObject,QSize,QPoint,QRect
import pygst
pygst.require("0.10")
import gst

class VideoDisplayWidget(QWidget):
	Signal_Camera_Set = pyqtSignal(int)
	def __init__(self, width,height, pipelines, parent = None):
		super(VideoDisplayWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.pipelines = pipelines
		self._cameraIndex = 0
		self.currentVideoStatus = 0
		self.Signal_Camera_Set.connect(self.Slot_Camera_Set)
		self.pixmap = QPixmap()

	def paintEvent(self, event):
		painter = QPainter()
		painter.begin(self)
		painter.drawPixmap(0,0, self.width,self.height, self.pixmap)
		painter.end()

	def Slot_Camera_Set(self, index):
		self._cameraIndex = index

	def Slot_Trigger_New_Frame(self, image, index):
		if self._cameraIndex == index:
			self.pixmap = image
			self.repaint()

#08.  Compass Widget - Signal
class CompassWidget(QLabel):
	Signal_Update_Heading = pyqtSignal(int)

	def __init__(self, width, height, compass_tick_spacing, font_size, parent = None):
		super(CompassWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.compass_tick_spacing = compass_tick_spacing # spacing between each tick
		self.font_size = font_size
		self.font_width = self.font_size
		self.Signal_Update_Heading.connect(self._Slot_Update_Heading)
		
		self._compass_mini_tick_length =  int(self.height/8) # length of a small tick
		self._compass_small_tick_length = int(self.height/4) # length of a small tick
		self._compass_large_tick_length = int(self.height/2) # length of a large tick
		self._compass_small_tick_rep = 9 # what a small tick represents
		self._compass_large_tick_rep = 45# what a large tick represents
		self._compass_heading_arrow_height = int(self.height/4)
		self._compass_names = ['N','NE','E','SE','S','SW','W','NW','N']
		self._heading = 0 # heading is counter clockwise

		self._color_border = QColor(255, 34, 3, 120)
		self._color_tick = QColor(255, 34, 3, 120)
		self._color_background = QColor(180, 180, 180, 120) #Qt.NoBrush
		self._color_font = QColor(255, 34, 3, 120)
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

#06.  Timer Widget - DONE
class TimerWidget(QLCDNumber):
	Signal_Update_Time = pyqtSignal(int)

	def __init__(self, width,height, time, parent = None):
		super(TimerWidget, self).__init__(5, parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.time = time
		self.setToolTip('This is the <b>Timer</b> widget')
		self.Signal_Update_Time.connect(self.Slot_Update_Time)
		
	def Slot_Update_Time(self, time):
		self.time = time
		timeStr = str(time/60)+":"+str(time%60)
		self.display(timeStr)

#05.  Accelerator Bar Widget - OPENCV - Cleaned
# Border, Background,ticks,bar(+/-)
class DriveBarWidget(QWidget):
	Signal_Update_Speed = pyqtSignal(int, float)
	Signal_Update_SentSpeed = pyqtSignal(int, float)
	def __init__(self, width,height, ruler_tick_size, ruler_tick_length, ruler_tick_num, large_tick_offset, tick_large_length, parent = None):
		super(DriveBarWidget, self).__init__(parent)
		#self.setToolTip('This is the <b>Drive Bar</b> widget')
		self.width = width
		self.height = height
		self.ruler_tick_size = ruler_tick_size    # how far apart small ticks are
		self.ruler_tick_length = ruler_tick_length# how long each small tick is
		self.ruler_tick_num = ruler_tick_num      # number of small ticks
		self.large_tick_offset = large_tick_offset# every X small tick is a large tick
		self.tick_large_length = tick_large_length # how long each large tick is
		
		self.resize(self.width,self.height)

		self._speeds = [0.0,0.0, 0.0,0.0]
		self._sentSpeeds = [0.0,0.0, 0.0,0.0]
		self._speedLeftPercent = 0
		self._speedRightPercent = 0
		self._MAX_WHEEL_SPEED = 100.0 #by percent
		self._sentSpeedLeftPercent = 0#speed sent by controller
		self._sentSpeedRightPercent = 0
		self.Signal_Update_Speed.connect(self._Slot_Update_Speed)
		self.Signal_Update_SentSpeed.connect(self._Slot_Update_SentSpeed)

		self.color_border = QColor(QColor(168, 34, 3))
		self.color_background = Qt.NoBrush
		self.color_ticks = QColor(168, 34, 3)
		self.color_barPositive = QColor(255, 175, 175)
		self.color_barNegative = QColor(175, 175, 255)
		self.color_sendSpeedBar = QColor(8,8,180)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self._drawDriveBar(event, qp)
		qp.end()
		
	def _drawDriveBar(self, event, qp):
		self._drawBorder(event,qp)
		self._drawSpeedBar(event,qp)
		self._drawTicks(event, qp)
		self._drawSentSpeed(event, qp)

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

	def _drawSentSpeed(self, event, qp):
		qp.setPen(self.color_sendSpeedBar)
		qp.setBrush(Qt.NoBrush)
		qp.drawLine(self.width/2-1+(self.width/2)*self._sentSpeedLeftPercent,0, self.width/2-1+(self.width/2)*self._sentSpeedLeftPercent,self.height/2)
		qp.drawLine(self.width/2-1+(self.width/2)*self._sentSpeedRightPercent,self.height/2, self.width/2-1+(self.width/2)*self._sentSpeedRightPercent,self.height)

	def _drawBorder(self, event, qp):
		qp.setPen(self.color_border)
		qp.setBrush(self.color_background)
		qp.drawLine(0,self.height/2, self.width, self.height/2) # draw middle(H) line
		qp.drawLine(self.width/2,0, self.width/2,self.height) # draw middle(V) line
		qp.drawRect(0,0, self.width-1, self.height-1)

	def _Slot_Update_Speed(self, param, value):
		#print 'Speed',param,value
		self._speeds[param] = value
		if param == 0 or param == 2:
			self._speedLeftPercent = (self._speeds[0]/self._MAX_WHEEL_SPEED+self._speeds[2]/self._MAX_WHEEL_SPEED)/2.0
		else:
			self._speedRightPercent = (self._speeds[1]/self._MAX_WHEEL_SPEED+self._speeds[3]/self._MAX_WHEEL_SPEED)/2.0
		#print 'update Speed:',param, value,self._speedLeftPercent,self._speedRightPercent
		self.repaint()

	def _Slot_Update_SentSpeed(self, param, value):
		#print 'sentSpeed',param,value
		self._sentSpeeds[param] = value
		if param == 0 or param == 2:
			self._sentSpeedLeftPercent = (self._sentSpeeds[0]/self._MAX_WHEEL_SPEED+self._sentSpeeds[2]/self._MAX_WHEEL_SPEED)/2.0
		else:
			self._sentSpeedRightPercent = (self._sentSpeeds[1]/self._MAX_WHEEL_SPEED+self._sentSpeeds[3]/self._MAX_WHEEL_SPEED)/2.0
		#print 'update Speed:',param, value,self._speedLeftPercent,self._speedRightPercent
		self.repaint()

#09.  Task List Widget - OPENCV - Cleaned
# Border, background, font
class TaskListWidget(QLabel):
	Signal_Update_Time = pyqtSignal(int)
	Signal_Set_Phase = pyqtSignal(int)
	Signal_Clear_Tasks = pyqtSignal()
	Signal_Add_Task = pyqtSignal(QString, QString, int, bool)
	Signal_Task_Done = pyqtSignal(int)
	Signal_Task_Not_Done = pyqtSignal(int)
	Signal_Repaint = pyqtSignal()

	def __init__(self, margin,phase1,phase2,phase3,fontBase, numRows, parent = None):
		super(TaskListWidget, self).__init__(parent)
		self.setToolTip('This is the <s>Task List</s> widget')
		self.margin = margin
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
		self.Signal_Repaint.connect(self._Slot_Repaint)
		self._current_task = 1 #1 = header; -1 = no header
		self._index_list = []
		self._task_list = []
		self._time_list = []
		self._is_done = []

		self._color_border = QColor(168, 34, 3)
		self._color_background = QColor(168,34,3, 100)
		self._color_font = QColor(255,255,255)
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
				qp.drawText((self.phase1)*self.fontWidth+self.margin, (self.fontHeight*1.5)*(j+1), self._task_list[i])
				if self._is_done[i]:
					qp.drawLine((self.phase1)*self.fontWidth+self.margin, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2, 
								(self.phase1)*self.fontWidth+self.margin, (self.fontHeight*1.5)*(j+1)-self.fontHeight/2)
			if self._phase >= 3:
				if (self._is_done[i] or i == self._current_task) and self._time_list[i] >= 0:
					qp.drawText(self.margin+(self.phase1+self.phase2)*self.fontWidth, (self.fontHeight*1.5)*(j+1), str(self._time_list[i]/60)+":"+str(self._time_list[i]%60))
			j += 1

	def _Slot_Repaint(self):
		self.repaint()

	def _Slot_Update_Time(self, time):
		# print 'time:',time,' ',str(self._current_task),' ',self._is_done[self._current_task]
		self._time = time
		if self._current_task < len(self._is_done):
			self._time_list[self._current_task] = self._time
		self.repaint()

	def _Slot_Set_Phase(self, phase):
		#print 'phase',phase
		if 1 <= phase <= 3:
			self._phase = phase
		else:
			if self._phase < 3:
				self._phase += 1
			else:
				self._phase = 1

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
		self._current_task = 1 #1 = header; -1 = no header
		self.repaint()

	def _Slot_Add_Task(self, index, task, time, isDone):
		self._index_list.append(index)
		self._task_list.append(task)
		self._time_list.append(time)
		self._is_done.append(isDone)
		self.repaint()

	def _Slot_Task_Done(self, index):
		# print str(index),' ',str(self._current_task)
		index = self._current_task
		if index < len(self._is_done):
			self._is_done[index] = True
			self._time_list[index] = self._time
			self._current_task += 1
			#print index,len(self._is_done),len(self._time_list),self._current_task
			#print index,self._is_done[index],self._time_list[index],self._current_task
		self.repaint()

	def _Slot_Task_Not_Done(self, index):
		index = self._current_task
		if index > 1:
			self._current_task -= 1
			self._is_done[index-1] = False
			self._time_list[index-1] = -1
			#print index,self._is_done[index-1],self._time_list[index-1],self._current_task
		self.repaint()
		
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
		self._crosshair_type = self.CROSSHAIR_TYPE_CROSS
		self.Signal_Crosshair_Type.connect(self._Slot_Crosshair_Type)

		self._color1 = QColor(255,125,125,125)
		self._color2 = QColor(125,125,255,255)
		
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
	Signal_Rotate_Angle = pyqtSignal(float)
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
		self.Signal_Rotate_Angle.connect(self._Slot_Rotate_Angle)
		self.Signal_Update_TiltX.connect(self._Slot_Update_TiltX)
		self.Signal_Update_TiltY.connect(self._Slot_Update_TiltY)
		self._tiltX = 0
		self._tiltY = 0
		self.rotation = 0
		self.angle = 0

		self.fontSize = self.height/10.0
		self.fontMargin = self.fontSize*1.5

		self._color_border = QColor(168, 34, 3)
		self._color_background = QColor(168, 34, 3, 50)
		self._color_font = QColor(255, 255, 255)
		self._color_foward = QColor(95,95,255)
		self._color_down_foward = QColor(95,255,95)
		self._color_rover_edge = QColor(255,125,125)
		self._font = QFont('Decorative', self.fontSize)

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
		qp.drawText(self.margin, self.height-self.margin, 'H:'+str(self.rotation)+" A:"+str(self.angle))
		qp.drawText(self.margin, self.height-self.margin-self.fontMargin, 'X'+str(self._tiltX))
		qp.drawText(self.margin, self.height-self.margin-self.fontMargin*2, 'Y'+str(self._tiltY))

	def _Slot_Rotate_Base(self, rotation):
		self._roverPicture.rotateBase(rotation)
		self.rotation = rotation
		self.repaint()
	def _Slot_Rotate_Angle(self, angle):
		self.angle = angle
		self._roverCamera.angle = self.angle
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
			self.angle = 0
			self.resize(self.width,self.height)

		def paintEvent(self, e):
			qp = QPainter()
			qp.begin(self)
			self._drawPic(qp)
			qp.end()

		def _drawPic(self, qp):
			if self.angle < 0:
				angle = self.angle * -1
				qp.setPen(self.parent._color_down_foward)
				qp.setBrush(self.parent._color_down_foward)
			else:
				angle = self.angle
				qp.setPen(self.parent._color_foward)
				qp.setBrush(self.parent._color_foward)
			length = (-(self.height/2-10))*((1-(angle)/90.0))
			length = (-(self.height/2-10))*((1-(angle)/90.0))
			thickness = self.width/20
			qp.drawRect(self.width/2-thickness/2,self.height/2,thickness,length)

#12.  HUD Overlay - OPENCV
class HUDOverlayWidget(QWidget):
	Signal_Insert_Nav_Point = pyqtSignal(float,float, QString)
	Signal_Clear_Nav_Point = pyqtSignal()
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
		self.Signal_Clear_Nav_Point.connect(self.Slot_Clear_Nav_Point)
		self.Signal_Update_Rover_GPS.connect(self.Slot_Update_Rover_GPS)
		self.Signal_Move_Camera_X.connect(self.Slot_Move_Camera_X)
		self.Signal_Move_Camera_Y.connect(self.Slot_Move_Camera_Y)


		self._color_edge = QColor(255,0,0)
		self._color_background = QColor(255,0,0, 120)
		self._color_font = QColor(255,0,0, 200)
		self.fontSize = 12
		self._font = QFont('times', self.fontSize)
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
			qp.drawText(self._nav_points[i].icon[3].x()-(self.fontSize/2.0)*1,self._nav_points[i].icon[3].y(), self._nav_points[i].tag)
			qp.drawText(self._nav_points[i].icon[1].x()-(self.fontSize/2.0)*3,self._nav_points[i].icon[1].y()+self.fontSize, str(self._nav_points[i].distance)[:5]+"m")# draw distance 

	def update_nav_relative(self):
		for i in range(len(self._nav_points)):
			self._nav_points[i].get_distance(self._rover_nav.longitude,self._rover_nav.latitude)
			self._nav_points[i].get_heading(self._rover_nav.longitude,self._rover_nav.latitude)
			# print 'nav',i,':',self._nav_points[i].longitude, ' ',self._nav_points[i].latitude,' ', self._nav_points[i].headingAngle,' ', self._nav_points[i].distance,self._nav_points[i].tag
	def Slot_Clear_Nav_Point(self):
		del self._nav_points[:]

	def Slot_Insert_Nav_Point(self, longitude,latitude,tag):
		self._nav_points.append(self.NavPoint(longitude,latitude, tag))
		self._nav_points[len(self._nav_points)-1].get_distance(self._rover_nav.longitude,self._rover_nav.latitude)
		self._nav_points[len(self._nav_points)-1].get_heading(self._rover_nav.longitude,self._rover_nav.latitude)
		# print 'nav',len(self._nav_points)-1,':',self._nav_points[len(self._nav_points)-1].longitude, ' ',self._nav_points[len(self._nav_points)-1].latitude,' ', self._nav_points[len(self._nav_points)-1].headingAngle,' ', self._nav_points[len(self._nav_points)-1].distance, tag
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
		# print 'heading:',heading, self._camera_heading, '<', -180+self._fov_width
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

# iwconfig wlan0
# ping ip/port
class SignalQualityWidget(QWidget):
	Signal_Update_Link_Quality = pyqtSignal(str)
	Signal_Update_Signal_Strength = pyqtSignal(str)
	Signal_Update_Ping = pyqtSignal(str)
	def __init__(self, width,height,fontWidth, parent=None):
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
		self._font_width  = fontWidth
		self._font_height = self._font_width

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
		qp.drawRect(0,0, self.width-1,self.height-1)

		qp.setPen(self._color_Text)
		qp.setFont(QFont('Decorative', self._font_height))
		qp.drawText(0,self.height-self.height/4, self._draw_Link_Quality)
		qp.drawText(self._font_width*7,self.height-self.height/4, self._draw_Signal_Strength)
		qp.drawText(self._font_width*7+self._font_width*8, self.height-self.height/4, self._draw_Ping)

	def _Slot_Update_Link_Quality(self, linkQuality):
		self._draw_Link_Quality = "LQ"+linkQuality
		#self.repaint()
	def _Slot_Update_Signal_Strength(self, sigStr):
		self._draw_Signal_Strength = "SS"+sigStr
		#self.repaint()
	def _Slot_Update_Ping(self, ping):
		self._draw_Ping = "P"+ping
		#self.repaint()

class MinimapWidget(QWidget):
	Signal_Set_Size = pyqtSignal(int,int)
	Signal_Clear_Nav_Point = pyqtSignal()
	Signal_Nav_Point = pyqtSignal(float,float, QString)
	Signal_Clear_Nav_Route = pyqtSignal()
	Signal_Nav_Route = pyqtSignal(float,float, float,float)
	Signal_View_GPS = pyqtSignal(float,float)
	Signal_Update_Rover_GPS = pyqtSignal(float,float)
	Signal_Update_Camera_Heading = pyqtSignal(int, float)
	Signal_Update_Camera_Angle = pyqtSignal(int, float)
	Signal_Update_Rover_Heading = pyqtSignal(float)
	Signal_Update_Map_Type = pyqtSignal(int)#0=Topo 1=Aerial
	Signal_Repaint = pyqtSignal()
	#Signal_Increase_Zoom = pyqtSignal(float)
	def __init__(self, width, height, roverRadius, parent = None):
		super(MinimapWidget, self).__init__(parent)
		self.setToolTip('This is the <b>Message</b> widget')
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
	#-------------------------------            Options            -------------------------------
		self._Map_Types = 0
		self._zoom = 1.0
	#-------------------------------            Load Maps            -------------------------------
		self.message_text = "OperatorGUI"
		self._topoQuads = []# img,W,E,S,N,x,y
		#NE,SW
		path = ""
		# topographic and elevation map
		# MAIN SITES
		self._topoQuads.append([QImage(path+'ProjectARMES/Resources/q3128_DRG24k-c.tiff'), -110.87571203540497, -110.75070541499849,38.37498125909681, 38.49998105277704, 4485.0,5697.0])
		self._topoQuads.append([QImage(path+'ProjectARMES/Resources/q3128_NED10.tiff'), -110.87571203540497, -110.75070541499849,38.37498125909681, 38.49998105277704, 1188.0,1511.0])
		# # High Resolution Aerial Maps of two zones
		# self._topoQuads.append([QImage(path+'resources/4-6299290481.tiff'), -110.812607741442,-110.749856262211,38.4165500165271,38.4584500229691, 5466,4637])
		# self._topoQuads.append([QImage(path+'resources/6-6982888891.tiff'), -110.81260758019,-110.74985647746,38.3748833857857,38.4167833239577, 5470,4637])
		
		#self._topoQuads.append([QImage(path+'/resources/export.tiff'), -110.875071935311,-110.74985647746, 38.3748833857857,38.5000833797906, 11935,11937])
		#HANKSVILLE
		# self._topoQuads.append([QImage(path+'ProjectARMES/Resources/hanksville/q3229_DRG24k-c.tif'), -110.75070841581692, -110.62570298853876, 38.24998330999908, 38.37498247798552, 4501,5703])
		# self._topoQuads.append([QImage(path+'ProjectARMES/Resources/hanksville/12SWH240440.tif'), -110.7254, -110.6794,38.343699999999544, 38.379899999999544, 16000,16000])
		'''
		
		'''
		#self._topoQuads.append([QImage(path+'ProjectARMES/Resources/q3129_NED10.tiff'), -110.87571203540497, -110.75070541499849,38.37498125909681, 38.49998105277704, 1602,1597])
		self._PIXEL_COORD = []
		for i in range(len(self._topoQuads)):
			self._PIXEL_COORD.append([(self._topoQuads[i][2] - self._topoQuads[i][1])/self._topoQuads[i][5], (self._topoQuads[i][4] - self._topoQuads[i][3])/self._topoQuads[i][6]])
			# print self._PIXEL_COORD[i]
	#-------------------------------            Display Vars            -------------------------------
		self.baseDegreePixelRatio = [self._PIXEL_COORD[0][0],self._PIXEL_COORD[0][1]]# 1 image pixel:x degrees
		self._midCoord = [self._topoQuads[0][1]+(self._topoQuads[0][2]-self._topoQuads[0][1])/2,self._topoQuads[0][3]+(self._topoQuads[0][4]-self._topoQuads[0][3])/2]# center coordinates
		self._pixelDegree = [self._PIXEL_COORD[0][0],self._PIXEL_COORD[0][1]]
		self._halfWidth = [self._pixelDegree[0]*self.width/2,self._pixelDegree[1]*self.height/2]# degrees from center
		# Left,Right, Down,Up Edges
		self._degreeBorder = [self._midCoord[0]-self._halfWidth[0],self._midCoord[0]+self._halfWidth[0],
							self._midCoord[1]-self._halfWidth[1],self._midCoord[1]+self._halfWidth[1]]
		self._midCoordPixel = [0,0]
	#-------------------------------            Navigation Vars            -------------------------------
		self._nav_points = []
		self._route_points = []
	#-------------------------------            Rover Vars            -------------------------------
		#self._camera_heading = 0
		#self._camera_fov = 60
		#self._camera_fov_length = self.width/10.0
		self._rover_longitude = self._midCoord[0]#self._longitude
		self._rover_latitude = self._midCoord[1]#self._latitude
		self._rover_heading = 0
	#-------------------------------            Signals & Slots            -------------------------------
		self.Signal_Set_Size.connect(self.Slot_Set_Size)
		self.Signal_Clear_Nav_Point.connect(self.Slot_Clear_Nav_Point)
		self.Signal_Nav_Point.connect(self.Slot_Nav_Point)
		self.Signal_Clear_Nav_Route.connect(self.Slot_Clear_Nav_Route)
		self.Signal_Nav_Route.connect(self.Slot_Nav_Route)
		self.Signal_View_GPS.connect(self.Slot_View_GPS)
		self.Signal_Update_Rover_GPS.connect(self.Slot_Update_Rover_GPS)
		self.Signal_Update_Camera_Heading.connect(self.Slot_Update_Camera_Heading)
		self.Signal_Update_Camera_Angle.connect(self.Slot_Update_Camera_Angle)
		self.Signal_Update_Rover_Heading.connect(self.Slot_Update_Rover_Heading)
		#self.Signal_Increase_Zoom.connect(self.Slot_Increase_Zoom)
		self.Signal_Update_Map_Type.connect(self.Slot_Update_Map_Type)
		self.Signal_Repaint.connect(self.Slot_Repaint)

		self._headingLength = 10
		self._roverRadius = roverRadius
		self._pixmap = QPixmap(self.width,self.height)
		self._isOpaque = 0
		self._fontSize = 10
		self._color_border = QColor(168,0,0)
		self._color_font = QColor(0,0,0)
		self._color_navpt = QColor(255,0,0)
		self._color_navroute = QColor(0,0,255)
		self._color_heading = QColor(0,0,0)
		self._cameras_fov = [[60,10.0,QColor(255,0,0, 120)]]
		self._cameras_headings = [[0.0,0.0],[0.0,0.0],[0.0,0.0]]
		self._roverCameras = []
		for i in range(len(self._cameras_fov)):
			self._roverCameras.append(MinimapWidget.RoverFOVWidget(self.width,self.height,self._cameras_fov[i][0],self._cameras_fov[i][1], self))
			self._roverCameras[i]._color_fov = self._cameras_fov[i][2]
			self._roverCameras[i].move(self.width/2-self._roverCameras[i].width/2,self.height/2-self._roverCameras[i].height/2)
		self._roverHeading = MinimapWidget.RoverHeadingWidget(self.width,self.height, self._roverRadius,self._headingLength, self)
		self._roverHeading.move(self.width/2-self._roverHeading.width/2,self.height/2-self._roverHeading.height/2)


	def resizeMap(self):
		for i in range(len(self._cameras_fov)):
			self._roverCameras[i].move(self.width/2-self._roverCameras[i].width/2,self.height/2-self._roverCameras[i].height/2)
		self._roverHeading.move(self.width/2-self._roverHeading.width/2,self.height/2-self._roverHeading.height/2)
		self.Slot_View_GPS(self._midCoord[0],self._midCoord[1])

	def decimal_to_pixel(self, decimal, division):
		result = (decimal) / (division*self._zoom)# self.width/2
		return result
	def long_to_pixel_x(self, longitude):
		result = (longitude) / (self._PIXEL_COORD[0][0]*self._zoom)# self.width/2
		return result
	def lati_to_pixel_y(self, latitude):
		result = ((latitude) / (self._PIXEL_COORD[0][1]*self._zoom))# self.height/2
		return result

	def mapOpaque(self,num):
		self._isOpaque = num

	def paintEvent(self, event):
		qp = QPainter()
		self._pixmap = QPixmap(self.width,self.height)
		qp.begin(self._pixmap)
		qp.translate(self.width/2, self.height/2)
		qp.setPen(self._color_border)
		qp.setBrush(self._color_border)
		qp.drawRect(-self.width/2,-self.height/2, self.width-1,self.height-1)
		self.drawMap(event, qp)
		self.drawNavPoints(event, qp)
		self.drawNavRoutes(event, qp)
		qp.end()
		qp.begin(self)
		qp.setOpacity(0.5)
		qp.setPen(self._color_border)
		qp.setBrush(Qt.NoBrush)
		qp.drawRect(0,0, self.width-1,self.height-1)
		qp.drawPixmap(0,0, self.width,self.height, self._pixmap)
		qp.setPen(self._color_font)
		qp.setBrush(Qt.NoBrush)
		'''
		# High 1538.22m
		# Low: 1320.82m
		1538.22-1320.82 = 217.4
		'''
		#print 'mid:',self._midCoordPixel[0],self._midCoordPixel[1]
		pixel = self._topoQuads[0][0].pixel(self._midCoordPixel[0],self._midCoordPixel[1])
		pixel = 1320.82 + (217.4 * (pixel/4294967295.0))# get elevation: min + elevation * greyscalePercent
		qp.drawText(10,10, " Z="+str(self._zoom)+" H="+str(self._rover_heading)+" Ele="+str(pixel) + " gps:"+ str(self._rover_longitude) + " : " + str(self._rover_latitude))
		#qp.drawText(10,10, str(self._zoom)+" "+str(self._midCoord[0])+str(self._midCoord[1]))
		qp.setOpacity(self._isOpaque)
		qp.end()
		

	def drawMap(self, event, qp):
		#for i in range(len(self._topoQuads)):
		if True:
			i = self._Map_Types
			# self._midCoord = [0.0,0.0]# center degree
			# self._pixelDegree = [2.7872156166439423e-05, 2.194133643676049e-05]# width/height
			# # max degrees from center(long/lati)
			# self._halfWidth = [self._pixelDegree[0]*self.width/2,self._pixelDegree[1]*self.height/2]# degrees from center
			# # Left,Right, Down,Up Edges
			# self._degreeBorder = [self._midCoord[0]-self._halfWidth[0],self._midCoord[0]+self._halfWidth[0],
			# 					self._midCoord[1]-self._halfWidth[1],self._midCoord[1]+self._halfWidth[1]]

			# print "midCoord",self._midCoord
			# print "pixelDegree",self._pixelDegree
			# print "halfWidth", self._halfWidth
			# print "degreeBorder", self._degreeBorder
			# img,W,E,S,N,x,y
			LongShows = False
			if self._topoQuads[i][1] <= self._degreeBorder[1]:# map[left] <= right Widget Edge: map shows
				LongShows = True
			if self._topoQuads[i][2] >= self._degreeBorder[0]:# map[Right] >= left widget Edge: map shows
				LongShows = True
			LatiShows = False
			if self._topoQuads[i][3] <= self._degreeBorder[3]:# map[down] <= top Widget Edge: map shows
				LatiShows = True
			if self._topoQuads[i][4] >= self._degreeBorder[2]:# map[up] <= down Widget Edge: map shows
				LatiShows = True
			# print "Long/LatiShows",LongShows,LatiShows
		#---------------        Phase 2         ---------------
			if LongShows and LatiShows:#show map
				coordEdges = [255,255,255,255]#L,R,D,U
				#left edge
				if self._degreeBorder[0] <= self._topoQuads[i][1] <= self._degreeBorder[1]:# Left map edge showing
					coordEdges[0] = self._topoQuads[i][1]
					#print "Left edge showing"
				elif self._topoQuads[i][1] <= self._degreeBorder[0] <= self._topoQuads[i][2]:# Inside map
					coordEdges[0] = self._degreeBorder[0]
					#print "no Left edge"
				#right edge
				if self._degreeBorder[0] <= self._topoQuads[i][2] <= self._degreeBorder[1]:# Right map edge showing
					coordEdges[1] = self._topoQuads[i][2]
					#print "Right edge showing"
				elif self._topoQuads[i][1] <= self._degreeBorder[1] <= self._topoQuads[i][2]:# Inside map
					coordEdges[1] = self._degreeBorder[1]
					#print "no right edge"
				#down edge
				if self._degreeBorder[2] <= self._topoQuads[i][3] <= self._degreeBorder[3]:# Down map edge showing
					coordEdges[2] = self._topoQuads[i][3]
					#print "down edge showing"
				elif self._topoQuads[i][3] <= self._degreeBorder[2] <= self._topoQuads[i][4]:# Inside map
					coordEdges[2] = self._degreeBorder[2]
					#print "no down edge"
				#up edge
				if self._degreeBorder[2] <= self._topoQuads[i][4] <= self._degreeBorder[3]:# Down map edge showing
					coordEdges[3] = self._topoQuads[i][4]
					#print "up edge showing"
				elif self._topoQuads[i][3] <= self._degreeBorder[3] <= self._topoQuads[i][4]:# Inside map
					coordEdges[3] = self._degreeBorder[3]
					#print "no up edge"
				# print "coordEdges:",coordEdges
				# print "center of map",coordEdges[0]+(coordEdges[1]-coordEdges[0])/2,":",coordEdges[2]+(coordEdges[3]-coordEdges[2])/2
			#---------------        Phase 3         ---------------
			# Get map offset from top left Corner of the widget
				shiftDegreeLong = 0
				if self._degreeBorder[0] <= coordEdges[0]:# Left check
					shiftDegreeLong = coordEdges[0] - self._degreeBorder[0]
				shiftDegreeLati = 0
				if self._degreeBorder[3] >= coordEdges[3]:
					shiftDegreeLati = self._degreeBorder[3] - coordEdges[3]
				# print "shiftDegreeLong:",self._degreeBorder[0] <= coordEdges[0],coordEdges[0]," - ",self._degreeBorder[0]," = ",shiftDegreeLong
				# print "shiftDegreeLati:",self._degreeBorder[3] >= coordEdges[3],coordEdges[3]," - ",self._degreeBorder[3]," = ",shiftDegreeLati
			#---------------        Phase 4         ---------------
			# Get map area offset from top left corner of the map
				shiftMapDegreeLong = 0
				if self._topoQuads[i][1] <= coordEdges[0]:
					shiftMapDegreeLong = coordEdges[0] - self._topoQuads[i][1]
				shiftMapDegreeLati = 0.0
				if self._topoQuads[i][4] >= coordEdges[3]:
					shiftMapDegreeLati = self._topoQuads[i][4] - coordEdges[3]
				# print "shiftMapDegreeLong:",self._topoQuads[i][1] <= coordEdges[0],coordEdges[0]," - ",self._topoQuads[i][1]," = ",shiftMapDegreeLong
				# print "shiftMapDegreeLati:",self._topoQuads[i][4] >= coordEdges[3],coordEdges[3]," - ",self._topoQuads[i][4]," = ",shiftMapDegreeLati#," = ",coordEdges[3] - self._topoQuads[i][4]
			#---------------        Phase 5         ---------------
			# Convert widget shift to pixels
				shiftDegreeX = shiftDegreeLong / self._pixelDegree[0] #convert map widget offset to pixels
				shiftDegreeY = shiftDegreeLati / self._pixelDegree[1]
				shiftMapDegreeX = shiftMapDegreeLong / self._PIXEL_COORD[self._Map_Types][0] #convert image offset to pixels
				shiftMapDegreeY = shiftMapDegreeLati / self._PIXEL_COORD[self._Map_Types][1]
				mapPixel = []# map display size(pixels)
				mapPixel.append((coordEdges[1]-coordEdges[0])/self._PIXEL_COORD[self._Map_Types][0])
				mapPixel.append((coordEdges[3]-coordEdges[2])/self._PIXEL_COORD[self._Map_Types][1])
				widgetMapPixel = []# size of the widget(pixels)
				widgetMapPixel.append((coordEdges[1]-coordEdges[0])/self._pixelDegree[0])
				widgetMapPixel.append((coordEdges[3]-coordEdges[2])/self._pixelDegree[1])
				# print "shiftDegreeX",shiftDegreeLong," / ",self._pixelDegree[0],"=",shiftDegreeX
				# print "shiftDegreeY",shiftDegreeLati," / ",self._pixelDegree[1]," = ",shiftDegreeY
				# print "shiftMapDegreeX",shiftMapDegreeLong," / ",self._pixelDegree[0]," = ",shiftMapDegreeX
				# print "shiftMapDegreeY",shiftMapDegreeLati," / ",self._pixelDegree[1]," = ",shiftMapDegreeY
				# print "mapPixel",mapPixel
				# print "widgetMapPixel",widgetMapPixel
				# print "center of map:",shiftMapDegreeX+mapPixel[0],shiftMapDegreeY+mapPixel[1]
			#---------------        Phase 6         ---------------
			# Draw map onto screen
				mapRect = QRect(shiftMapDegreeX,shiftMapDegreeY, mapPixel[0],mapPixel[1])
				self._midCoordPixel = [shiftMapDegreeX+mapPixel[0]/2,shiftMapDegreeY+mapPixel[1]/2]
				widgetRect = QRect(-self.width/2+shiftDegreeX,-self.height/2+shiftDegreeY, widgetMapPixel[0],widgetMapPixel[1])
				#qp.drawImage(widgetRect, self._topoQuads[self._Map_Types][0], mapRect)
				qp.drawImage(widgetRect, self._topoQuads[i][0], mapRect)
				#print "-------------------------------------------------------------------------"

	def drawNavPoints(self, event,qp):
		qp.setPen(self._color_navpt)
		qp.setBrush(self._color_navpt)
		qp.setFont(QFont('times',15))
		diviX = self._PIXEL_COORD[self._Map_Types][0]
		diviY = self._PIXEL_COORD[self._Map_Types][1]
		for i in range(len(self._nav_points)):
			symbol_size = 4
			long_screen = self._nav_points[i][0] - self._midCoord[0]
			lati_screen = self._midCoord[1] - self._nav_points[i][1]
			pixel_x = self.decimal_to_pixel(long_screen,self._PIXEL_COORD[self._Map_Types][0])
			pixel_y = self.decimal_to_pixel(lati_screen,self._PIXEL_COORD[self._Map_Types][1])
			qp.drawEllipse(pixel_x-symbol_size/2,pixel_y-symbol_size/2, symbol_size,symbol_size)
			qp.drawText(pixel_x,pixel_y,self._nav_points[i][2])
			#print 'pt',i,' ',long_screen,' ',lati_screen,' ',pixel_x,' ',pixel_y

	def drawNavRoutes(self, event,qp):
		qp.setPen(self._color_navroute)
		qp.setBrush(self._color_navroute)
		diviX = self._PIXEL_COORD[self._Map_Types][0]
		diviY = self._PIXEL_COORD[self._Map_Types][1]
		for i in range(len(self._route_points)):
			long_screen = []
			lati_screen = []
			pixel_x = []
			pixel_y = []
			for j in range(2):
				long_screen.append(self._midCoord[0] - self._route_points[i][j][0])
				lati_screen.append( self._midCoord[1] - self._route_points[i][j][1])
				pixel_x.append(self.decimal_to_pixel(long_screen[j],self._PIXEL_COORD[self._Map_Types][0]))
				pixel_y.append(self.decimal_to_pixel(lati_screen[j],self._PIXEL_COORD[self._Map_Types][1]))
			#print 'draw route:',-pixel_x[0],pixel_y[0], -pixel_x[1],pixel_y[1]
			qp.drawLine(-pixel_x[0],pixel_y[0], -pixel_x[1],pixel_y[1])

	def repaintRover(self):
		long_screen = self._midCoord[0] - self._rover_longitude
		lati_screen = self._midCoord[1] - self._rover_latitude
		pixel_x = self.decimal_to_pixel(long_screen,self._PIXEL_COORD[self._Map_Types][0])
		pixel_y = self.decimal_to_pixel(lati_screen,self._PIXEL_COORD[self._Map_Types][1])
		self._roverHeading.move(self.width/2+-1*pixel_x-self._roverHeading.width/2,self.height/2+pixel_y-self._roverHeading.height/2)
		for i in range(len(self._cameras_fov)):
			self._roverCameras[i].move(self.width/2+-1*pixel_x-self._roverCameras[i].width/2,self.height/2+pixel_y-self._roverCameras[i].height/2)
		
	def Slot_Repaint(self):
		self.repaint()
	def Slot_Set_Size(self,width,height):
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.repaintRover()
		self.repaint()
	def Slot_Clear_Nav_Point(self):
		self._nav_points = []
		self.repaint()
	def Slot_Nav_Point(self, longitude,latitude, name):
		self._nav_points.append([longitude,latitude,name])
		self.repaint()
	def Slot_Clear_Nav_Route(self):
		self._route_points = []
		self.repaint()
	def Slot_Nav_Route(self, longitude,latitude, longitude2,latitude2):
		self._route_points.append([[longitude,latitude], [longitude2,latitude2]])
		#print 'navroute',longitude,latitude, longitude2,latitude2
		self.repaint()
	def Slot_View_GPS(self, longitude,latitude):
		#print "Signal_View_GPS",longitude,latitude
		self._midCoord[0] = longitude
		self._midCoord[1] = latitude
		self._pixelDegree = [self._PIXEL_COORD[self._Map_Types][0]*self._zoom,self._PIXEL_COORD[self._Map_Types][1]*self._zoom]
		self._halfWidth = [self._pixelDegree[0]*self.width/2,self._pixelDegree[1]*self.height/2]# degrees from center
		self._degreeBorder = [self._midCoord[0]-self._halfWidth[0],self._midCoord[0]+self._halfWidth[0],
							self._midCoord[1]-self._halfWidth[1],self._midCoord[1]+self._halfWidth[1]]
		# print "midCoord",self._midCoord
		# print "pixelDegree",self._pixelDegree
		# print "halfWidth", self._halfWidth
		# print "degreeBorder", self._degreeBorder
		self.repaintRover()
		self.repaint()
	def Slot_Update_Rover_GPS(self, longitude,latitude):
		self._rover_longitude = longitude
		self._rover_latitude = latitude
		self.repaintRover()
		self.repaint()
	def Slot_Update_Camera_Heading(self, index, heading):
		self._cameras_headings[index][0] = heading
		self._roverCameras[index].Signal_Heading.emit(self._cameras_headings[index][0])
		self.repaint()
	def Slot_Update_Camera_Angle(self, index, Angle):
		self._cameras_headings[index][1] = Angle
		self._roverCameras[index].Signal_Angle.emit(self._cameras_headings[index][1])
		self.repaint()
	def Slot_Update_Rover_Heading(self, heading):
		self._rover_heading = heading
		self._roverHeading.Signal_Heading.emit(self._rover_heading)
	def Slot_Update_Map_Type(self, num):
		self._zoom = self._zoom * self._PIXEL_COORD[self._Map_Types][0]/self._PIXEL_COORD[num][0]
		self._Map_Types = num
		self.Slot_View_GPS(self._midCoord[0],self._midCoord[1])
	def zoomIncrease(self, zoom):
		self.zoomSet(self._zoom + zoom)
	def zoomDecrease(self, zoom):
		self.zoomSet(self._zoom - zoom)
	def zoomSet(self, zoom):
		if 0.1 <= zoom:
			self._zoom = zoom
			self.Slot_View_GPS(self._midCoord[0],self._midCoord[1])
	def popNavPoint(self):
		if len(self._nav_points) > 0:
			return self._nav_points.pop()
	def popNavRoute(self):
		if len(self._route_points) > 0:
			return self._route_points.pop()

	class RoverFOVWidget(QWidget):
		Signal_Heading = pyqtSignal(float)
		Signal_Angle = pyqtSignal(float)
		def __init__(self, width,height,fov,length, parent = None):
			super(MinimapWidget.RoverFOVWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.resize(self.width,self.height)
			self._camera_fov = fov
			self._camera_fov_length = length
			self._camera_heading = 0
			self._camera_angle = 0
			self.Signal_Heading.connect(self.Slot_Heading)
			self.Signal_Angle.connect(self.Slot_Angle)
			self._color_fov = QColor(255, 34, 3)

		def paintEvent(self, event):
			qp = QPainter()
			qp.begin(self)
			qp.translate(self.width/2, self.height/2)
			qp.rotate(self._camera_heading+self.parent._rover_heading)
			self.draw_rover_heading(event, qp)
			qp.end()

		def draw_rover_heading(self, event, qp):
			qp.setPen(self._color_fov)
			qp.setBrush(self._color_fov)
			qp.setFont(QFont('Decorative', 20))
			#(1-self._camera_angle/90.0)*
			endX = self._camera_fov_length*math.cos((self._camera_fov/180.0)*math.pi)
			endY = self._camera_fov_length*math.sin((self._camera_fov/180.0)*math.pi)
			qp.drawLine(0,0, endX,-endY) #left line
			qp.drawLine(0,0, -endX,-endY) #right line
			#qp.setPen(QColor(255, 34, 3,100))
			qp.setBrush(self._color_fov)
			qp.drawPolygon(QPoint(0,0),QPoint(endX/2,-endY/2),QPoint(-endX/2,-endY/2),QPoint(0,0))

		def Slot_Heading(self, heading):
			self._camera_heading = heading
			#print str(self._camera_heading)
			self.repaint()
		def Slot_Angle(self, angle):
			self._camera_angle = angle
			self.repaint()

	class RoverHeadingWidget(QWidget):
		Signal_Heading = pyqtSignal(float)
		def __init__(self, width,height, roverCircle,pointerLength, parent = None):
			super(MinimapWidget.RoverHeadingWidget, self).__init__(parent)
			self.parent = parent
			self.width = width
			self.height = height
			self.resize(self.width,self.height)
			self.pointerLength = pointerLength
			self.roverCircle = roverCircle
			self._rover_heading = self.parent._rover_heading
			self.Signal_Heading.connect(self.Slot_Heading)
			self._color_heading = self.parent._color_heading

		def paintEvent(self, event):
			qp = QPainter()
			qp.begin(self)
			qp.translate(self.width/2, self.height/2)
			qp.rotate(self._rover_heading)
			self.draw_rover_heading(event, qp)
			qp.end()

		def draw_rover_heading(self, event, qp):
			qp.setPen(self._color_heading)
			qp.setBrush(self._color_heading)
			qp.setFont(QFont('Decorative', 20))

			# self.pointerLength = 10
			# self.roverCircle = 5/2
			qp.drawEllipse(-self.roverCircle/2,-self.roverCircle/2, self.roverCircle,self.roverCircle) # draw rover dot
			pt1 = QPoint(-self.roverCircle/2,-self.roverCircle/2)
			pt2 = QPoint(self.roverCircle/2,-self.roverCircle/2)
			pt3 = QPoint(0,-self.pointerLength)
			qp.drawPolygon(pt1,pt2,pt3,pt1)

		def Slot_Heading(self, heading):
			self._rover_heading = heading
			self.repaint()

class CameraStatusWidget(QWidget):
	Signal_Camera_Status = pyqtSignal(int) #0=stop 1=pause 2:ready 3:playing
	Signal_Camera_Set = pyqtSignal(str)
	Signal_MenuCamera_Set = pyqtSignal(str)
	def __init__(self, width,height, parent = None):
		super(CameraStatusWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width,self.height)
		self.cameraName = ''
		self.menuCameraName = ''

		self._margin_ratio = 5.0
		self.margin = self.height/self._margin_ratio
		self.rectSize = self.height-self.margin*2
		self.pauseSize = self.rectSize
		self.readySize = self.rectSize
		self.playSize = self.rectSize
		self.fontSize = self.rectSize/2

		self.currentVideo = 0 #current video to show
		self.currentVideoStatus = 0 #current video status (stop,pause,start)
		self.Signal_Camera_Status.connect(self.Slot_Camera_Status)
		self.Signal_Camera_Set.connect(self.Slot_Camera_Set)
		self.Signal_MenuCamera_Set.connect(self.Slot_MenuCamera_Set)
		self._color_border = QColor(255,0,0)
		self._color_background = QColor(255,0,0,100)
		self._color_Icon = QColor(255,255,255,200)

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawBackground(event, qp)
		self.drawWidget(event, qp)
		qp.end()

	def drawBackground(self, event, qp):
		qp.setPen(self._color_border)
		qp.setBrush(self._color_background)
		qp.drawRect(0,0, self.width-1,self.height-1)

	def drawWidget(self, event, qp):
		qp.setPen(self._color_Icon)
		qp.setBrush(self._color_Icon)
		qp.setFont(QFont('Decorative', self.fontSize, 75))
		if self.currentVideoStatus == gst.STATE_NULL:# stop
			qp.drawRect(self.margin,self.margin, self.rectSize,self.rectSize)
		elif self.currentVideoStatus == gst.STATE_PAUSED:#=pause
			qp.drawRect(self.margin,self.margin, self.pauseSize/4,self.pauseSize)
			qp.drawRect(self.margin+(self.pauseSize/4)*2,self.margin, self.pauseSize/4,self.pauseSize)
		elif self.currentVideoStatus == gst.STATE_READY:#=ready
			qp.drawEllipse(self.margin,self.margin, self.readySize,self.readySize)
		elif self.currentVideoStatus == gst.STATE_PLAYING:#=playing
			pt1 = QPoint(self.margin,self.margin)
			pt2 = QPoint(self.margin,self.margin+self.playSize)
			pt3 = QPoint(self.margin+self.playSize,self.margin+self.playSize/2)
			qp.drawPolygon(pt1,pt2,pt3)
		qp.drawText(self.rectSize+self.margin*2,self.rectSize/1.5, self.cameraName)
		qp.drawText(self.rectSize+self.margin*2,self.rectSize*1.5, self.menuCameraName)

	def Slot_Camera_Status(self, status):
		self.currentVideoStatus = status
		self.repaint()
	def Slot_Camera_Set(self, cameraName):
		self.cameraName = cameraName
		self.repaint()
	def Slot_MenuCamera_Set(self, cameraName):
		self.menuCameraName = cameraName
		self.repaint()

class GPSDisplayWidget(QWidget):
	Signal_Update_GPS = pyqtSignal(float,float, float)
	def __init__(self, width,height,font, parent=None):
		super(GPSDisplayWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.Signal_Update_GPS.connect(self._Slot_Update_GPS)
		self._color_Edge = QColor(100,0,100,200)
		self._color_Background = QColor(100,0,100,100)
		self._color_Text = QColor(255,255,255)
		self._draw_GPS = ""
		self._font_width  = font
		self._font_height = self._font_width
		# print 'gps display init'

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		self.drawText(qp)
		qp.end()

	def drawText(self, qp):
		# print 'redraw display'
		qp.setPen(self._color_Edge)
		qp.setBrush(self._color_Background)
		qp.drawRect(0,0, self.width-1,self.height-1)

		qp.setPen(self._color_Text)
		# qp.setFont(QFont('Decorative', self._font_height))
		qp.drawText(0,self._font_height*1.5, self._draw_GPS)

	def _Slot_Update_GPS(self, longitude,latitude, altitude):
		self._draw_GPS = str(longitude)+","+str(latitude)+":"+str(altitude)+'m'
		self.repaint()

class HelpWidget(QWidget):
	def __init__(self, width,height,fontSize, parent=None):
		super(HelpWidget, self).__init__(parent)
		self.width = width
		self.height = height
		self.resize(self.width, self.height)
		self.color_Edge = QColor(255,255,0)
		self.color_Background = QColor(255,0,0,200)
		self.color_Text = QColor(255,255,255)
		self.font_size = fontSize
		self.font = QFont('Decorative', self.font_size)
		self.margin = self.font_size
		self.num = 1
		self.helpText = []

	def paintEvent(self, e):
		qp = QPainter()
		qp.begin(self)
		self.drawBorder(qp)
		self.drawHelp(qp)
		qp.end()

	def drawBorder(self, qp):
		qp.setPen(self.color_Edge)
		qp.setBrush(self.color_Background)
		qp.drawRect(0,0, self.width-1,self.height-1)

	def drawHelp(self, qp):
		qp.setPen(self.color_Text)
		qp.setBrush(Qt.NoBrush)
		qp.setFont(self.font)
		for i in range(len(self.helpText)):
			self.printHelp(qp, self.helpText[i])
		self.num = 1

	def printHelp(self, qp, text):
		qp.drawText(self.margin, self.num*self.font_size*1.5, text)
		self.num += 1

	def addHelpText(self, string):
		self.helpText.append(string)