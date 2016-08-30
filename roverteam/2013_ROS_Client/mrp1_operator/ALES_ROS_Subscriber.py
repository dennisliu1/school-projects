#!/usr/bin/env python
""" ales_ROS_Subscriber
    Dennis Liu 2013
"""

import pygame

from Wrapper import Wrapper

import roslib;
roslib.load_manifest('yurt_base')
roslib.load_manifest('mrp1_operator')
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from mrp1_operator.msg import Conveyor,Drive


ALES_DRIVE_MAX = 65278.0 #0xFEFE
ALES_FRONT_CONVEYOR_MAX = 65278.0 #0xFEFE
ALES_BACK_CONVEYOR_MAX = 65278.0 #0xFEFE
ALES_VERTICAL_CONVEYOR_MAX = 65278.0 #0xFEFE

def callback_conveyor(conveyor_msg):
    print 'conveyor_msg:',conveyor_msg.front_conveyor, conveyor_msg.back_conveyor, conveyor_msg.vertical
    if conveyor_msg.front_conveyor, conveyor_msg.back_conveyor, conveyor_msg.vertical
    pub_master_conveyor.publish(conveyor_msg)
def callback_drive(drive_msg):
    print 'drive_msg:',drive_msg.left, drive_msg.right
    if (drive_left is not drive_msg.left) or (drive_right is not drive_msg.right):
        if (drive_left is not drive_msg.left):
            drive_left = drive_msg.left
        if (drive_right is not drive_msg.right):
            drive_right = drive_msg.right
        pub_master_drive.publish(Drive(drive_left,drive_right))

def listener():
    #variables
    drive_left = 0.0
    drive_right = 0.0
    vertical = 0.0
    back_conveyor = 0.0
    front_conveyor = 0.0

    rospy.init_node('listener', anonymous=True)#this node's name = listener
    sub_conveyor = rospy.Subscriber("conveyor", Conveyor, callback_conveyor)# listen to the 'chatter' topic
    sub_drive = rospy.Subscriber("drive", Drive, callback_drive)
    pub_master_conveyor = rospy.Publisher('master_conveyor', Conveyor) #publish to chatter topic
    pub_master_drive = rospy.Publisher('master_drive', Drive)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass