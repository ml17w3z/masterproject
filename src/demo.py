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


#global variable that would be used to store the distance and the recognised points
distance_data = 0
rec_points = 0
bridge = CvBridge()

#initialise move it moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

#initialise ros node. First param is the name of the node we are creating
rospy.init_node('motion_planning', anonymous = True)

#create robot commander objects to be used to control the robot
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("arm")

gripper = moveit_commander.MoveGroupCommander("gripper")

torso = moveit_commander.MoveGroupCommander("arm_torso")

#Move the arm position of the robot in it's initial state, which is folded. This is used so that the robot can safely go to one place to another with less chance of hitting its arm
def tuck():
    #clear the pose target
    group.clear_pose_targets()
    #these are the target configuration of the arm and torso
    movement_one = [0.5217855019860682, 0.686551571949809, -2.2787982492530254, 2.2331901736024324, -0.5878297665061805, -1.4632443148703125, 0.00040730042547565404]
    movement_two = [0.19999864091101927, -1.338797212423665, -0.1999251151660797, 1.9381888897276625, -1.570070632083512, 1.3698331534959323, 3.5244041729498576e-06]

    #get the current configuration
    group_variable_values = group.get_current_joint_values()

    #overwrite the current configuration
    for i in range (0,7):
        group_variable_values[i] = movement_one[i]
    group.set_joint_value_target(group_variable_values)
    #execute the new configuration of the robot's arm
    group.go(wait=True)
    rospy.sleep(2)

    #overwrite the current configuration
    for i in range (0,7):
        group_variable_values[i] = movement_two[i]
    #set and then execute the new configuration
    group.set_joint_value_target(group_variable_values)
    group.go(wait=True)
    rospy.sleep(2)

    #open the gripper on the arm
    gripper_open()


#this function is used to close the gripper on the arm of the robot
def gripper_close():
    gripper.clear_pose_targets()
    gripper_values = gripper.get_current_joint_values()
    #if they are both zero, then they would be in contact with each other
    #top claw
    gripper_values[0] = 0.01
    #bottom claw
    gripper_values[1] = 0.01
    gripper.set_joint_value_target(gripper_values)
    gripper.go(wait=True)


#this function is used to open the gripper
def gripper_open():
    gripper.clear_pose_targets()
    gripper_values = gripper.get_current_joint_values()
    gripper_values[1] = 0.04
    #set and then execute the configuration
    gripper.set_joint_value_target(gripper_values)
    gripper.go(wait=True)


#this would be used to print the current info of the robot. This was mainly used to find out the target pose of the Robot
#change the plan and then execute in Rviz and then print the configuration
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




# this would be used so that the robot does not hit its arm when tucking it
# param seconds_time_ten is the number of seconds the robot moves, but multiply by 10
def move_back(seconds_times_ten):
    #This would be the topic to publish to
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    desired_vel = Twist()

    #negative so that it moves backwards
    desired_vel.linear.x = -0.06

    #50 = 5 seconds
    for i in range (0, seconds_times_ten):
        pub.publish(desired_vel)
        rate.sleep()



# this is used to move the robot forward, in order to go throught the door, or simply reposition
# param seconds_time_ten is the number of seconds the robot moves, but multiply by 10
def move_forward(seconds_times_ten):
    #this is the topic that would be used
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    #rospy.init_node('Pulling', anonymous=True)
    rate = rospy.Rate(10)
    desired_vel = Twist()

    desired_vel.linear.x = 0.06

    #50 = 5 seconds
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


#This is the motion that the robot needs to perform in order to open the door
def door_opening_motion():
    #clear any pose target if there's any
    torso.clear_pose_targets()

    #these are the configurations. They are taken from rviz and using the print_info function i have created
    position_arm =  [0.14168025775768234, 0.9146888551871903, 0.7323415997640232, -2.308657743286636, 2.184890031719192, 1.0495603317807412, 1.4984467691403154, -0.26656024194813455]
    lift_arm =  [0.3440452760642088, 0.3548658658720356, 0.07890462789437969, -3.0442108848831504, 1.7570033305778914, -0.36314858420079865, -1.5458134720009733, 1.7929571011742276]
    postion_arm_next_to_door = [0.33803900512138807, 0.6579493821383187, 0.1312261178346006, -2.846090124819133, 1.2776927496824246, -0.5987195510828967, -1.2248894039295166, 1.5539307736993706]
    bring_arm_down = [0.33219240870089056, 0.16884391912444308, 0.9728224311936273, -1.5744730196271872, 1.0978310488651584, 0.9198050786851137, -1.5016325272046043, 1.447666734204124]
    down_and_position = [0.3321906668870037, 0.1473813620816733, 0.9625324033087104, -1.6057689363940506, 1.1211835535764072, 0.9171587218970183, -1.379190143631706, 1.458888365646147]
    reposition =  [0.25256950140987144, 0.34863289282533483, 0.8298163064784649, -1.4423003575456228, 1.0892469160036873, 0.8053830100860457, -1.2703528345936812, 1.4789013071704273]
    reposition_two = [0.2467299272399419, 0.37911953476017146, 0.8154407644072279, -1.3970977835113594, 1.0790401486717922, 0.8012233238796966, -1.366850205778821, 1.4729350240721253]
    reposition_three = [0.2962245620444472, 0.2661895917177084, 0.8993831623892952, -1.4853972218425184, 1.1079693350149213, 0.868855803830126, -1.4384570598775595, 1.4611698354374072]
    reposition_four  = [0.29622875588876574, 0.02002356905882241, 0.5897433481845393, -1.9878008992509626, 1.480799312104014, 0.4935938250276797, -1.5426333879058554, 1.7192168380433905]
    position_open = [0.3160097827309733, 1.4521291975578352, -1.4076427623089582, -1.8806871890173742, 2.3348936072886612, 0.2207172590784321, 1.5507881262073049, -0.8665187888918924]
    test_two = [0.24056124552509428, 0.27081779034279574, 0.876623310645777, -1.4606849009957674, 1.3124354971441008, 0.834376200256087, -1.2667022312943121, 1.5844324966687608]
    position_open_two = [0.343350342758816, 2.728817630903709, 0.5594718370739251, 0.5945070121721265, 2.2106713392272948, -0.13040408615668841, -1.1186472591271945, -1.47763591825772]
    position_open_three = [0.344087796658188, 2.1559836391543996, 0.5382098232465857, 0.43789191105658887, 1.7247943982235618, 0.3771883858135139, -1.3410393017979256, -2.032665538506677]
    position_open_four = [0.29135738938035327, 1.5323953358322235, -0.06696385303758845, 1.46975330860261, 1.1375302157806058, -1.6516560564612277, 0.703843175655896, 0.20259694877858614]
    test1=[0.3260717295026062, 0.19558007601461025, 0.6774893981026446, -1.804476374915172, 1.7267004409192843, 0.4063647399866346, -1.1669357823060151, 2.0689308363278407]
    #get the current joint values
    torso_values = torso.get_current_joint_values()

    #this is to position the arm
    for i in range (0,8):
        torso_values[i] = position_arm[i]
    torso.set_joint_value_target(torso_values)
    
    torso.go(wait=True)
    print('1')
    rospy.sleep(2)

    #this is for further positioning and lifting the torso
    for i in range (0,8):
        torso_values[i] = lift_arm[i]
    torso.set_joint_value_target(torso_values)
    
    torso.go(wait=True)
    print('2')
    rospy.sleep(2)

    #this is for further positioning and lifting the torso
    for i in range (0,8):
        torso_values[i] = postion_arm_next_to_door[i]
    torso.set_joint_value_target(torso_values)
    
    torso.go(wait=True)
    print('3')
    rospy.sleep(2)

    #this is used to bring the arm down, so that it is directly above the door handle
    for i in range (0,8):
        torso_values[i] = bring_arm_down[i]
    torso.set_joint_value_target(torso_values)
    
    torso.go(wait=True)
    print('4')
    rospy.sleep(2)


    #this is used to bring the arm down, so that it is close to the door handle
    for i in range (0,8):
        torso_values[i] = down_and_position[i]
    torso.set_joint_value_target(torso_values)
    torso.go(wait=True)
    print('5')
    rospy.sleep(2)

    #this is used to reposition the arm
    for i in range (0,8):
        torso_values[i] = reposition[i]
    torso.set_joint_value_target(torso_values)
    torso.go(wait=True)
    print('6')
    rospy.sleep(2)

    #this is used for further repositioning
    for i in range (0,8):
        torso_values[i] = test_two[i]
    torso.set_joint_value_target(torso_values)
    torso.go(wait=True)
    print('7')
    rospy.sleep(2)

    #close the gripper, move backwards for 4 seconds and then open the gripper again
    gripper_close()
    move_forward(150)
    gripper_open()
    for i in range (0,8):
        torso_values[i] = postion_arm_next_to_door[i]
    torso.set_joint_value_target(torso_values)
    torso.go(wait=True)
    print('8')
    rospy.sleep(2)
    rotate(-0.1)
    tuck()
    move_forward(200)

def door_pull():
    #clear any pose target if there's any
    torso.clear_pose_targets()
    init_arm =  [0.14168025775768234, 0.9146888551871903, 0.7323415997640232, -2.308657743286636, 2.184890031719192, 1.0495603317807412, 1.4984467691403154, -0.26656024194813455]
    test1=[0.3198289810991829, 0.4306707928946887, 0.6195794520432658, -1.6663918740404764, 1.4500504793856779, 0.3659979482556537, -1.2175518032122836, 1.9705115439694936]
    test2=[0.3135994627375901, 2.6055457384288907, 0.8947451961045036, 1.0029079362034485, 1.3068370582668436, 2.0425727013800845, -0.9123666717143486, 1.6652893156129718]
    #get the current joint values
    torsoarm = torso.get_current_joint_values()
    #this is to position the arm
    for i in range (0,8):
        torsoarm[i] = init_arm[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    rospy.sleep(2)
    
    

    for i in range(0,8):
        torsoarm[i]=test2[i]
    torso.set_joint_value_target(torsoarm)
    torso.go(wait=True)
    print('1')
    rospy.sleep(2)
    gripper_close() 
    move_back(40)
    gripper_open()

    

    

#This function is used to listen to the topics created in other python script
def listener():
    sub = rospy.Subscriber('Distance', Float32, callback)
    sub_rec_point = rospy.Subscriber('RecPoints', Int8, callback_rec)


#This function would be used for the subscriber for distance
def callback(msg):
    global distance_data
    distance_data = msg.data


#This function would be used for the subscriber for sub_rec_point
def callback_rec(msg):
    global rec_points
    rec_points = msg.data


#This function would be used to take pictures
def screenshoot():
    image_subscriber = rospy.Subscriber('/xtion/rgb/image_raw', Image, callback_pic)
    time.sleep(0.1)


#this function would be used for image_subscriber
def callback_pic(data):
    try:
        #conver the image to cv image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        print e
    #save the image
    cv2.imwrite("door2.png", cv_image)
    

def image_match():
    #open the images
    #image_cam = cv2.imread('/home/bryce/tiago-lib/tiago_ws/door.png',0)
    image_cam = cv2.imread('door2.png',0)
    image_tem = cv2.imread('222.png',0)
    #create an orb extractor
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
    good_match = 0#this variable is used to count the number of good matches
    good = []
    for m_n in matches:#each thing in the list must be checked if there are 2 neighbours
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
        #if the object in front of it is between 0.4 and 0.55, take a picture of it
        #the RecPoints topic created at the other file constantly checks number of matched key points
        #if it's more than 0, then we know that there is a door, therefore we can stop taking picture
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
    
    if re>=10:
        door_pull()
    else:
        #door_opening_motion()
        print('stop')


    

    
    
            
            

    