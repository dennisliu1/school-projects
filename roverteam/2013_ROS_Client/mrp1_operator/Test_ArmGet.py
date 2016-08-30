#!/usr/bin/env python
""" ales_ROS_Subscriber
    Dennis Liu 2013
"""
import time

# import pygame

import roslib;
#roslib.load_manifest('yurt_base')
roslib.load_manifest('mrp1_operator')
import rospy
# from geometry_msgs.msg import Twist, Vector3
# from std_msgs.msg import String
from mrp1_operator.msg import Compass

class TestCompass():
    def __init__(self):
        self.ALES_compass = 0
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_ALES_Compass = rospy.Publisher('ALES_Send_Compass', Compass)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Compass", Compass, self.callback_ALES_Compass)

    def sendMessage(self):
        self.pub_ALES_Compass.publish(Compass(self.ALES_compass))

    def incrementData(self):
        self.ALES_compass += 1
        if self.ALES_compass >= 360:
            self.ALES_compass = 0

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.ALES_compass == 0:
                self.isRunning = False
            print 'compass',self.ALES_compass
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass