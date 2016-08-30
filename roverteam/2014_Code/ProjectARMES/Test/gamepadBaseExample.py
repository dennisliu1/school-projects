import time
import pygame
from pygame.locals import *
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

class InputManager:
    def __init__(self):
        print 'joystick:',pygame.joystick.get_count()
        self.init_joystick()
        self.isRunning = True

    # Set joystick information.
    # The joystick needs to be plugged in before this method is called (see main() method)
    def init_joystick(self):
        joystick = pygame.joystick.Joystick(0)# num = ID
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
                        self.sendCommand('axis:'+str(i)+':'+str(axis)+'\n')
            if eventType == JOYBALLMOTION:
                for i in range(self.numBalls):
                    ball = self.joystick.get_ball(i)
                    if self.prevBalls[i] != ball:
                        self.prevBalls[i] = ball
                        self.sendCommand('ball:'+str(i)+':'+str(ball)+'\n')
            if eventType == JOYHATMOTION:
                for i in range(self.numHats):
                    hat = self.joystick.get_hat(i)
                    if self.prevHats[i] != hat:
                        self.prevHats[i] = hat
                        self.sendCommand('hat:'+str(i)+':'+str(hat)+'\n')
            if eventType == JOYBUTTONUP:
                for i in range(self.numButtons):
                    button = self.joystick.get_button(i)
                    if self.prevButtonsUp[i] != button:
                        self.prevButtonsUp[i] = button
                        self.sendCommand('button_up:'+str(i)+':'+str(button)+'\n')
            if eventType == JOYBUTTONDOWN:
                for i in range(self.numButtons):
                    button = self.joystick.get_button(i)
                    if self.prevButtonsUp[i] != button:
                        self.prevButtonsUp[i] = button
                        self.sendCommand('button_down:'+str(i)+':'+str(button)+'\n')

    def sendCommand(self, sendCmd):
        sock.sendto(sendCmd, (UDP_IP, UDP_PORT))

def main():
    delay = 0.001

    pygame.init()
    input_manager = InputManager()
    while input_manager.isRunning:
        # input_manager.get_events()
        input_manager.doStuff()
        time.sleep(delay)


if __name__ == '__main__':
    main()