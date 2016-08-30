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
from mrp1_operator.msg import Signals

class TestSignals():
    def __init__(self):

        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_BASE_Signals = rospy.Publisher('BASE_OperatorGUI_Status',OperatorGUI_Status)

    def sendMessage(self):
        self.pub_BASE_Signals.publish(Signals(self.signalQuality,self.signalLevel))

    def incrementData(self):
        

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.quality2 <= 1:
                self.isRunning = False
            print 'signal:',self.quality1,self.quality2,self.level
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass