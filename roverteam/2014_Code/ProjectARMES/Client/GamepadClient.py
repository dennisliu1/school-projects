import sys
import time
import threading
import pygame
from pygame.locals import *

class InputManager:
	def __init__(self, gamepad=0):
		print 'joystick:',pygame.joystick.get_count()
		if pygame.joystick.get_count() > 0:
			self.gamepad = gamepad
			self.init_joystick()
			self.isRunning = True
			self.thread = self.GamepadThread(self, 1, self.doStuff, 0.001)
		else:
			print 'no gamepad detected!'

	def startThreads(self):
		if pygame.joystick.get_count() > 0:
			self.thread.start()

	def endThreads(self):
		self.isRunning = False

	# Set joystick information.
	# The joystick needs to be plugged in before this method is called (see main() method)
	def init_joystick(self):
		joystick = pygame.joystick.Joystick(self.gamepad)# num = ID
		joystick.init()
		self.joystick = joystick
		self.joystick_name = joystick.get_name()
		self.numAxis = self.joystick.get_numaxes()
		self.numBalls = self.joystick.get_numballs()
		self.numButtons = self.joystick.get_numbuttons()
		self.numHats = self.joystick.get_numhats()
		print 'name    :',self.joystick_name
		print 'axis    :',self.numAxis
		print 'balls   :',self.numBalls
		print 'buttons :',self.numButtons
		print 'hats    :',self.numHats
		self.prevAxis = list()
		for i in range(self.numAxis):
			self.prevAxis.append(0)
		self.prevBalls = list()
		for i in range(self.numBalls):
			self.prevBalls.append(0)
		self.prevButtonsUp = list()
		for i in range(self.numButtons):
			self.prevButtonsUp.append(0)
		self.prevButtonsDown = list()
		for i in range(self.numButtons):
			self.prevButtonsDown.append(0)
		self.prevHats = list()
		for i in range(self.numHats):
			self.prevHats.append(0)
		
	def doStuff(self):
		# print 'doStuff'
		for event in pygame.event.get():
			eventType = event.type
			# print 'event : ' + str(eventType)
			if eventType == JOYAXISMOTION:
				for i in range(self.numAxis):
					axis = self.joystick.get_axis(i)
					if self.prevAxis[i] != axis:
						self.prevAxis[i] = axis
						# self.axisEvent(i, axis)
				self.axisEvent2(self.prevAxis)
				# self.sendCmd()
			elif eventType == JOYBALLMOTION:
				for i in range(self.numBalls):
					ball = self.joystick.get_ball(i)
					if self.prevBalls[i] != ball:
						self.prevBalls[i] = ball
						self.ballEvent(i, ball)
			elif eventType == JOYHATMOTION:
				for i in range(self.numHats):
					hat = self.joystick.get_hat(i)
					if self.prevHats[i] != hat:
						self.prevHats[i] = hat
						self.hatEvent(i, hat)
			elif eventType == JOYBUTTONUP:
				for i in range(self.numButtons):
					button = self.joystick.get_button(i)
					if self.prevButtonsUp[i] != button:
						self.prevButtonsUp[i] = button
						self.buttonUpEvent(i, button)
			elif eventType == JOYBUTTONDOWN:
				for i in range(self.numButtons):
					button = self.joystick.get_button(i)
					if self.prevButtonsUp[i] != button:
						self.prevButtonsUp[i] = button
						self.buttonDownEvent(i, button)

	def axisEvent(self, num, axis):
		pass

	def axisEvent2(self, axis):
		pass

	def ballEvent(self, num, ball):
		pass

	def hatEvent(self, num, hat):
		pass

	def buttonUpEvent(self, num, button):
		pass

	def buttonDownEvent(self, num, button):
		pass

	def sendCmd(self):
		pass

	class GamepadThread(threading.Thread):
		def __init__(self, parent, num, doStuff, delay):
			threading.Thread.__init__(self)
			self.parent = parent
			self.num = num
			self.doStuff = doStuff
			self.delay = delay

		def run(self):
			print 'thread',self.num,':','started'
			isRunning = True
			while isRunning and self.parent.isRunning:
				self.doStuff()
				#print 'recv',self.num,':',msg
				time.sleep(self.delay)
			print 'thread',self.num,':','ended'


def main(gamepad=0):
	delay = 0.001

	pygame.init()
	input_manager = InputManager()
	input_manager.startThreads()
	# while input_manager.isRunning:
	# 	# input_manager.get_events()
	# 	input_manager.doStuff()
	# 	time.sleep(delay)

if __name__ == '__main__':
	main(int(sys.argv[1]))