#!/usr/bin/env python
""" ales_ROS_Subscriber
    Dennis Liu 2013
"""
import time

# import pygame
import os
import roslib;
#roslib.load_manifest('yurt_base')
roslib.load_manifest('mrp1_operator')
import rospy
# from geometry_msgs.msg import Twist, Vector3
# from std_msgs.msg import String
from mrp1_operator.msg import Ping

class TestPing():
    def __init__(self):
        self.cmd = "ping localhost -c1"
        self.ping = ""
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_BASE_Ping = rospy.Publisher('BASE_Ping', Ping)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Compass", Compass, self.callback_ALES_Compass)

    def sendMessage(self):
        self.pub_BASE_Ping.publish(Ping(self.ping))

    def incrementData(self):
        f = os.popen(self.cmd)
        now = f.read()
        now = now.splitlines()
        rlst = now[1]
        rlst = rlst[rlst.find("time=")+5:]
        self.ping = rlst
        print 'rslt:',rlst

    def start(self):
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass