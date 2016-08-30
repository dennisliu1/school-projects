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
from mrp1_operator.msg import GPSpt

class TestGPS():
    def __init__(self):
        self.ALES_GPS_Longitude = -80.0
        self.ALES_GPS_Latitude  = 43.75
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_ALES_GPS = rospy.Publisher('ALES_Send_GPS', GPSpt)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Compass", Compass, self.callback_ALES_Compass)

    def sendMessage(self):
        self.pub_ALES_GPS.publish(GPSpt(self.ALES_GPS_Latitude,self.ALES_GPS_Longitude))

    def incrementData(self):
        #W     E      S     N
        #-80.0,-79.5, 43.75,44.0
        self.ALES_GPS_Longitude += 0.001
        self.ALES_GPS_Latitude += 0.001

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            if self.ALES_GPS_Latitude >= 44.00 or self.ALES_GPS_Longitude >= -79.50:
                self.isRunning = False
            print 'GPS:',self.ALES_GPS_Longitude,self.ALES_GPS_Latitude
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass