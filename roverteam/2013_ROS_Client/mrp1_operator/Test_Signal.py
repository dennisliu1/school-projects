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
from mrp1_operator.msg import Signals

class TestSignals():
    def __init__(self):
        self.cmd = "iwconfig wlan0"
        self.signalQuality = ""
        self.signalLevel = ""
        rospy.init_node('listener', anonymous=True)#this node's name = listener
        self.pub_BASE_Signals = rospy.Publisher('ALES_Send_Signals', Signals)
        #self.sub_ALES_conveyor = rospy.Subscriber("ALES_Compass", Compass, self.callback_ALES_Compass)

    def sendMessage(self):
        #self.signalQuality = str(self.quality1)+"/"+str(self.quality2)
        #self.signalLevel = str(self.level)
        self.pub_BASE_Signals.publish(Signals(self.signalQuality,self.signalLevel))

    def incrementData(self):
        '''
        wlan0     IEEE 802.11abgn  ESSID:"IBMVISITOR"  
          Mode:Managed  Frequency:2.437 GHz  Access Point: 00:1F:26:2B:57:D1   
          Bit Rate=11 Mb/s   Tx-Power=15 dBm   
          Retry  long limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=36/70  Signal level=-74 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:0  Invalid misc:39   Missed beacon:0
        '''
        f = os.popen(self.cmd)
        now = f.read()
        now = now.splitlines()
        now = now[5]
        self.signalQuality = now[now.find("Link Quality=")+5+8:now.find("Signal level=")-2]
        self.signalLevel = now[now.find("Signal level=")+7+6:]

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        self.isRunning = True
        while self.isRunning:
            self.incrementData()
            self.sendMessage()
            print 'signal:',self.signalQuality, self.signalLevel
            time.sleep(1)

# if __name__ == '__main__':
#     try:
#         test = TestCompass()
#         test.start()
#     except rospy.ROSInterruptException:
#         pass