#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
import sys
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from time import sleep
import time


#this is used to detect the distance between door and robot
pub = rospy.Publisher('Distance', Float32, queue_size=10)
def listener():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    sub2 = rospy.Subscriber('Distance', Float32, callback2)
    rospy.spin()

def callback(msg):
    laser_reading_front = msg.ranges[333]
    pub.publish(laser_reading_front)

#we need the msg.data to learn the distance info
def callback2(msg):
    rospy.loginfo(msg.data)
    











if __name__ == '__main__':
    try :
        listener()
    except :
        pass
