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
from mrp1_operator.msg import PTZ

class Test():
    def __init__(self):
        self.ALES_Heading = 0
        self.ALES_angle = 0
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_ALES_PTZ = rospy.Publisher('ALES_PTZ', PTZ)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Heading", Compass, self.callback_ALES_Heading)

    def sendMessage(self):
        self.pub_ALES_PTZ.publish(PTZ(self.ALES_Heading,self.ALES_angle))

    def incrementData(self):
        self.ALES_Heading += 1
        if self.ALES_Heading >= 180:
            self.ALES_Heading = 0
        self.ALES_angle += 1
        if self.ALES_angle >= 90:
            self.ALES_angle = 0

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.ALES_Heading == 180:
                self.isRunning = False
            print 'PTZ:',self.ALES_Heading,self.ALES_angle
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass