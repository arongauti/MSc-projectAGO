#!/usr/bin/env python2.7  
import time
from typing import Container
import roslib
roslib.load_manifest('auto_labeller')
import rospy
import math
from math import radians
import tf
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Header
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import copy
import moveit_commander
import os, os.path
import moveit_msgs.msg
from random import random, randrange, randint
from datetime import date, datetime

#The box bounderies....
XMAX = 0.36 # 0.38
XMAX2 = 0.405
XMIN = 0.28
YMAX = 0.137
YMIN = -0.096
#Home position
HOMEX = 0.42272#0.3091
HOMEY = 0.00726#0.0048
HOMEZ = 0.42751#0.4234

locerror = 0.01 # max error in movement
moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
folder_path = "/home/lab/Pictures/"
counter = 0
error = 0.05 # error from the camera, since it is tilted
suctionCup = 0.14#0.045 #decreas if want to go further down 0.034 length from endeffector to end of suction cup red suction 5.5cm
dropSpace = 0.015
# green 4.5cm
def endPos(msg,rot): 
    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    #Find difference between camera and suction end
    diff=PointStamped()
    diff.header.frame_id = str(msg.header.frame_id) #camera_color_optical_frame
    diff.header.stamp = rospy.Time(0)
    diff.point.x=position.x  #z
    diff.point.y=position.y #x
    diff.point.z=position.z #y 
    p=listener.transformPoint("panda_suction_end",diff)

    pickPoint=PointStamped()
    pickPoint.header.frame_id = "panda_suction_end"
    pickPoint.header.stamp = rospy.Time(0)
    pickPoint.point.x=p.point.x
    pickPoint.point.y=p.point.y
    pickPoint.point.z=p.point.z-suctionCup
    p = listener.transformPoint("panda_link0",pickPoint)
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = rot[0]
    pose_target.orientation.y = rot[1]
    pose_target.orientation.z = rot[2]
    pose_target.orientation.w = rot[3]
    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    pose_target.position.z = p.point.z
    currentbox = [quat.x, quat.y, quat.z, quat.w]
    print(currentbox)
    return pose_target, quat     
    
def captureImage(currentbox):
    global counter
    msg = rospy.wait_for_message(image_color, Image)
    color_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    imgname = str(counter).zfill(4)
    f= open(os.path.join(dirName, imgname+".txt"),"w+")
    if currentbox != 0:
        f.write("0 %.9f %.9f %.9f %.9f\r\n" % (float(currentbox.x),float(currentbox.y),float(currentbox.z),float(currentbox.w)))
    f.close() 
    cv2.imwrite(os.path.join(dirName, imgname+".png"), color_frame)
    counter = counter+1

def homePos():
    print("home pos")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = float(radians(0))
    group_variable_values[1] = float(radians(-25))
    group_variable_values[2] = float(radians(1))
    group_variable_values[3] = float(radians(-133))
    group_variable_values[4] = float(radians(0))
    group_variable_values[5] = float(radians(109))
    group_variable_values[6] = float(radians(45)) # 45 is straight
    group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    group.go(wait=True)


def moveAbove(pose_target):
    captureImage(currbox)
    before = pose_target.position.z
    
    pose_target.position.z = 0.20 + suctionCup
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    downspace = 0 
    pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
    pubSuc.publish(Bool(True))
    while(pressure.data > -0.1):
        if pose_target.position.z < 0.025+suctionCup or downspace > 0.035:
            homePos()
            rospy.sleep(0.3)
            pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
            if pressure.data > -0.1:
                pubSuc.publish(Bool(False))
                return 0
        else:        
            pose_target.position.z = before-downspace
            group.set_pose_target(pose_target)
            plan1 = group.plan()
            group.go(wait=True)
            downspace = downspace + 0.003
        pressure = rospy.wait_for_message("/vacuum/pressure", Float32)
        
        
    
    pose_target.position.z = before+downspace
    return 1

def moveToOtherBox(currpos): #move the item that is picked to a "locked" location...
    homePos()
    currpos.position.x = 0.45
    currpos.position.y = -0.42
    currpos.position.z = 0.43
    currpos.orientation.x = 0.99999
    currpos.orientation.y = 0.006
    currpos.orientation.z = 0.008
    currpos.orientation.w = 0.007
    group.set_pose_target(currpos)
    plan1 = group.plan()
    group.go(wait=True)
    pubSuc.publish(Bool(False))

def stateMachine(currState):
    global endPosition, start, currbox, dirName
    if currState == 0:
        folder = ""
        print("Create new dir")
        folder = raw_input("Folder name\n") # user creates a folder
        dirName = folder_path + "/" + folder
        try:
            # Create target Directory
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
            counter = 0
        except:
            print("Directory " , dirName ,  " already exists") 
        return 1
    elif currState == 1:
        #captureImage("")
        value = 0
        while value != 2: 
            value = input("Enter 2 to get coordinates:\n")
            value = int(value)
        return value
    elif currState == 2: 
        currPoint=PointStamped()
        currPoint.header.frame_id = "panda_suction_end"
        home = listener.transformPoint("panda_link0",currPoint)
        if ((abs(HOMEX-home.point.x)<locerror and abs(HOMEY-home.point.y)<locerror and abs(HOMEZ-home.point.z)<locerror)):
            try:
                (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
                msg = rospy.wait_for_message("/pandaposition", PoseStamped)
                rospy.loginfo("Received at goal message!")
                if(msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0 ):
                    return 6
                endPosition, currbox = endPos(msg,rot) # transforming
                rospy.loginfo(endPosition)
                if endPosition == "None":
                    
                    return 2
                else: 
                    emptycount = 0
                    return 3
                #print(endPosition)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("look up fail")    
        else:
            homePos()
            return 2

    elif currState == 3:
        start = time.time()
        pick=moveAbove(endPosition)
        if pick == 0:
            return 2
        rospy.sleep(0.2)
        homePos()
        return 4
        #stateMachine(4)        
        
    elif currState == 4:
        moveToOtherBox(endPosition)
        homePos()
        rospy.sleep(2)
        end = time.time()
        rospy.loginfo("TIME:")
        rospy.loginfo(end - start)
        return 2
    elif currState == 5:
        print(currState)
    elif currState == 6:
        print(currState)
        exitstate = -1
        homePos()
        pubSuc.publish(Bool(False))
        captureImage(0)
        while exitstate != 0: 
            exitstate = input("Enter 0 to begin again and create new directory:\n")
            exitstate = int(exitstate)
        print(exitstate)
        return 0
        
        

if __name__ == '__main__':
    emptycount = 0
    #rostopic pub  std_msgs/Bool true --once
    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1) # Talk to the festo ovem suction cup
    rospy.init_node('pick_and_place')
    listener = tf.TransformListener()
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10) # Talks to the moveit motion planner
    homePos()
    empty = input("Enter 0 to start the statemachine\n")
    empty = int(empty)
    now = datetime.now()
    # dd/mm/YY H:M:S
    dt_string = now.strftime("%d-%m-%Y-%H:%M")
    if empty == 0:
        currstate = stateMachine(empty)

    while not rospy.is_shutdown():
        currstate = stateMachine(currstate)
        
            

