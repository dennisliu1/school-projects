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
from mrp1_operator.msg import Astrobox

class Test():
    def __init__(self):
        self.num = 0
        self.numMax = 50
        self.ALES_Astrobox = [0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_ALES_Astrobox = rospy.Publisher('ALES_Astrobox', Astrobox)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Compass", Compass, self.callback_ALES_Compass)

    def sendMessage(self):
        self.pub_ALES_Astrobox.publish(Astrobox(self.ALES_Astrobox[0],self.ALES_Astrobox[1],self.ALES_Astrobox[2],self.ALES_Astrobox[3],self.ALES_Astrobox[4]))

    def incrementData(self):
        for i in range(len(self.ALES_Astrobox)):
            if self.ALES_Astrobox[i] == 0:
                self.ALES_Astrobox[i] = 50
            else:
                self.ALES_Astrobox[i] = 0
        self.num += 1

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.num >= self.numMax:
                self.isRunning = False
            print 'astrobox',self.ALES_Astrobox
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass