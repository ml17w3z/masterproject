#!/usr/bin/env python
from __future__ import division
import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
import time
from threading import Thread
from multiprocessing import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from time import sleep

#The robot arm operation funciton such as pull and push is learnt from Adrain Bonus and  moveit tutorial for python
#official moveit tutorial example code github website:https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel, the access to Adrain's oringal code is limited to memebers of robot club, please contact Adrain or me for code.

 #The part of image matching is learnt from the opencv tutorial, 
 #the link is: https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html#matcher

#firstly we need to set some global name
distance_data = 0
bridge = CvBridge()

#initialise move it moveit_commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning', anonymous = True)
#create moveit commander group and set group name
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("arm")

gripper = moveit_commander.MoveGroupCommander("gripper")

torso = moveit_commander.MoveGroupCommander("arm_torso")

#this function is used to (1)hold its arm back (2)keep the robot's front is clear
def holdarm():
    #clear the pose target
    group.clear_pose_targets()
 
    movement = [0.19999864091101927, -1.338797212423665, -0.1999251151660797, 1.9381888897276625, -1.570070632083512, 1.3698331534959323, 3.5244041729498576e-06]
    holdvalue = group.get_current_joint_values()

    for i in range (0,7):
        holdvalue[i] = movement[i]

    group.set_joint_value_target(holdvalue)
    group.go(wait=True)
    rospy.sleep(2)
    #open the gripper on the arm
    gripper_open()


#this function is used to close the gripper on the arm of the robot
def gripper_close():
    gripper.clear_pose_targets()
    gripper_values = gripper.get_current_joint_values()
    #smaller the value is, closer the two fingers are
    gripper_values[0] = 0.01
    gripper_values[1] = 0.01
    gripper.set_joint_value_target(gripper_values)
    gripper.go(wait=True)


#this function is used to open the gripper
def gripper_open():
    gripper.clear_pose_targets()
    gripper_values = gripper.get_current_joint_values()
    gripper_values[1] = 0.03
    gripper_values[0]=0.03
    gripper.set_joint_value_target(gripper_values)
    gripper.go(wait=True)



#this function is used to print the current information of robot arm
def GetInfo():

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names =robot.get_group_names()
    print "============ Robot Groups:", group_names

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


def move_back(seconds_times_ten):
    #This would be the topic to publish to
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    desired_vel = Twist()
    desired_vel.linear.x = -0.06

    for i in range (0, seconds_times_ten):
        pub.publish(desired_vel)
        rate.sleep()



def move_forward(seconds_times_ten):
    #this is the topic that would be used
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    #rospy.init_node('Pulling', anonymous=True)
    rate = rospy.Rate(10)
    desired_vel = Twist()

    desired_vel.linear.x = 0.06

    for i in range (0, seconds_times_ten):
        pub.publish(desired_vel)
        rate.sleep()


# this is used to rotate the robot's base
#param ang is the speed in which the robot rotates
def rotate(ang):
    #this is the topic we would publish in order to do the movement
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    desired_vel = Twist()

    desired_vel.angular.z = ang

    for i in range (0, 30):
        pub.publish(desired_vel)
        rate.sleep()



def door_pull():
    #clear any pose target if there's any
    torso.clear_pose_targets()
    init_arm =  [0.14168025775768234, 0.9146888551871903, 0.7323415997640232, -2.308657743286636, 2.184890031719192, 1.0495603317807412, 1.4984467691403154, -0.26656024194813455]
    test1=[0.3198289810991829, 0.4306707928946887, 0.6195794520432658, -1.6663918740404764, 1.4500504793856779, 0.3659979482556537, -1.2175518032122836, 1.9705115439694936]
    #get the current joint values
    torsoarm = torso.get_current_joint_values()
    #this is to position the arm
    for i in range (0,8):
        torsoarm[i] = init_arm[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    rospy.sleep(2)
    
    

    for i in range(0,8):
        torsoarm[i]=test1[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    print('1')
    rospy.sleep(2)
    gripper_close()
    move_back(40)
    

def push():
    #clear any pose target if there's any
    torso.clear_pose_targets()
    init_arm =  [0.14168025775768234, 0.9146888551871903, 0.7323415997640232, -2.308657743286636, 2.184890031719192, 1.0495603317807412, 1.4984467691403154, -0.26656024194813455]
    test1=[0.3198289810991829, 0.4306707928946887, 0.6195794520432658, -1.6663918740404764, 1.4500504793856779, 0.3659979482556537, -1.2175518032122836, 1.9705115439694936]
    postion_arm_next_to_door = [0.33803900512138807, 0.6579493821383187, 0.1312261178346006, -2.846090124819133, 1.2776927496824246, -0.5987195510828967, -1.2248894039295166, 1.5539307736993706]
    #get the current joint values
    torsoarm = torso.get_current_joint_values()
    #this is to position the arm
    for i in range (0,8):
        torsoarm[i] = init_arm[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    rospy.sleep(2)
    
    for i in range(0,8):
        torsoarm[i]=test1[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    print('1')
    rospy.sleep(2)
    gripper_close() 
    move_forward(150)
    gripper_open()
    for i in range (0,8):
        torsoarm[i] = postion_arm_next_to_door[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    print('finish')
    rospy.sleep(2)
    holdarm()
    rotate(-0.15)
    move_forward(150)
    
        

    

#This function is used to listen to the topics created in other python script
def listener():
    sub = rospy.Subscriber('Distance', Float32, callback)
    


#This function is the callback for distance
def callback(msg):
    global distance_data
    distance_data = msg.data





#This function takes pictures
def screenshoot():
    image_subscriber = rospy.Subscriber('/xtion/rgb/image_raw', Image, callback_pic)
    time.sleep(0.1)


#this callback function is used to convert and store images
def callback_pic(data):
    try:
        #conver the image to cv image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        print e
    #we need to save it so that we can do the image matching
    cv2.imwrite("door.png", cv_image)
    

def image_match():
    #read the image
    #image_cam = cv2.imread('/home/bryce/tiago-lib/tiago_ws/door.png',0)
    image_cam = cv2.imread('door.png',0)
    image_tem = cv2.imread('111.png',0)
    #create an orb extractor
    #we can adjust the parameter values in it
    orb = cv2.ORB_create(6000, 1.2, nlevels=12, edgeThreshold=4)

    #find the keypoints and describe them
    camera_key_points, camera_key_description = orb.detectAndCompute(image_cam,None)
    tem_kp, tem_kd = orb.detectAndCompute(image_tem,None)

    FLANN_INDEX_LSH = 6

    index_params = dict(algorithm = FLANN_INDEX_LSH,
                        table_number = 6,
                        key_size=12,
                        multi_probe_level=1)
    search_params = dict(check=100)
    flann=cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(camera_key_description,tem_kd,k=2)
    # Apply ratio test
    good_match = 0
    good = []
    for m_n in matches:
        if len(m_n)!=2:
            continue
        (m,n)=m_n
        if m.distance < 0.75*n.distance:#if ratio is close to 1, they are equal
             good.append([m])
             good_match = good_match+1
    print(len(good))
    return len(good)




if __name__ == '__main__':
    

    while (1):
        listener()
        if distance_data > 0.8 or distance_data==0 :
            print('go forward')
            print distance_data
            move_forward(10)
        else:
            break
    #rotate back to the original position and go throught the door
    print('ok!')
   
    while (1):
        listener()
        print distance_data
        if 0.5 < distance_data < 0.8:
            print('lets open the door!')
            break

    #door_opening_motion()
    screenshoot()
    re=image_match()
    print('THe image matches are:',re)
    if re>10:
        print('This is a door with handle')
    else:
        print('It is not a door with handle')
    move_forward(25)
    #only open the door if the matching results are good
    if re>=10:
        #change to another function to switch push/pull
        #door_pull()
        push()
    else:
        print('stop')


    

    
    
            
            

    