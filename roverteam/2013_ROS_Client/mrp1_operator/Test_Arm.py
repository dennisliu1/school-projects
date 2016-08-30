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
from mrp1_operator.msg import Arm

class TestArm():
    def __init__(self):
        self.elbow1 = 0.0
        self.elbow2 = 0.0
        self.endeffector = 0.0
        self.end1 = 0.0
        self.end2 = 0.0
        self.end3 = 0.0
        self.end4 = 0.0
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_ALES_Compass = rospy.Publisher('BASE_Arm', Arm)

    def sendMessage(self):
        msg = Arm(elbow1,elbow2, endeffector,end1,end2,end3,end4)
        self.pub_ALES_Compass.publish(msg)

    def incrementData(self):
        self.elbow1 += 1.0
        self.elbow2 += 1.0
        self.endeffector += 1.0
        self.end1 += 1.0
        self.end2 += 1.0
        self.end3 += 1.0
        self.end4 += 1.0
        if self.elbow >30:
            self.elbow1 = 0.0
            self.elbow2 = 0.0
            self.endeffector = 0.0
            self.end1 = 0.0
            self.end2 = 0.0
            self.end3 = 0.0
            self.end4 = 0.0

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.elbow1 >= 30:
                self.isRunning = False
            print 'arm:',self.elbow1,self.elbow2,self.endeffector,self.end1,self.end2,self.end3,self.end4,self.end5)
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass