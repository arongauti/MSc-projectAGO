#!/usr/bin/env python  
import roslib
roslib.load_manifest('auto_labeller')
import rospy
import math
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
from std_msgs.msg import Header
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import copy
import moveit_commander
import moveit_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
# Instantiate CvBridge
bridge = CvBridge()

image_color = "/camera/color/image_raw"
folder_path = "/home/lab/Pictures/test/"
counter = 0
error = 0.05 # error from the camera, since it is tilted
suctionCup = 0.039 #decreas if want to go further down 0.034 length from endeffector to end of suction cup red suction 5.5cm
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
    
    """ pe = PoseStamped()
    pe.header.frame_id = "/panda_link0"
    pe.header.stamp = rospy.Time.now()
    pe.pose.orientation.x = rot[0]
    pe.pose.orientation.y = rot[1]
    pe.pose.orientation.z = rot[2]
    pe.pose.orientation.w = rot[3]
    pe.pose.position.x = p.point.x
    pe.pose.position.y = p.point.y
    pe.pose.position.z = p.point.z """
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = rot[0]
    pose_target.orientation.y = rot[1]
    pose_target.orientation.z = rot[2]
    pose_target.orientation.w = rot[3]
    pose_target.position.x = p.point.x
    pose_target.position.y = p.point.y
    pose_target.position.z = p.point.z
    
    return pose_target     
    
def captureImage(state):
    path = folder_path+"/"+state
    msg = rospy.wait_for_message(image_color, Image)
    color_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    imgbeforetext = "%s%d.png" %(path,counter)
    cv2.imwrite(imgbeforetext, color_frame)

def currPos(pos):
    locerror = 0.01 
    pickPoint=PointStamped()
    pickPoint.header.frame_id = "panda_suction_end"
    p = listener.transformPoint("panda_link0",pickPoint)
    x,y,z = pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
    while(not rospy.is_shutdown() and not(abs(x-p.point.x)<locerror and abs(x-p.point.x)<locerror and abs(z-p.point.z)<locerror)):
        p = listener.transformPoint("panda_link0",pickPoint)
        x,y,z = pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
        continue
    print("Robot in right place")

def homePos():
    #global moveit_msgs
    group.set_named_target("ready")
    plan1 = group.plan()
    group.go(wait=True)
    

def moveToOtherSide(pos):
    #pos.position.x = pos.position.x
    pos.position.y = -1*pos.position.y
    pos.position.z = pos.position.z+0.02
    moveAbove(pos)
    #pub.publish(pe)
    #currPos(pe)

def moveAbove(pose_target):
    before = pose_target.position.z

    pose_target.position.z = 0.30
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)

    pose_target.position.z = before
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)

    """  before = pos.pose.position.z
    pos.pose.position.z = 0.30
    pub.publish(pos)
    currPos(pos)
    pos.pose.position.z = before
    pub.publish(pos)
    currPos(pos) """

if __name__ == '__main__':
    pub = rospy.Publisher('keyboard', PoseStamped, queue_size=10)
    #rostopic pub  std_msgs/Bool true --once
    pubSuc = rospy.Publisher('/vacuum/set_suction_on', Bool, queue_size=1)
    rospy.init_node('pick_and_place')
    listener = tf.TransformListener()
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    while not rospy.is_shutdown():
        value = input("Enter 1 to get coordinates:\n")
        value = int(value)
        if value == 1:
            try:
                (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
                msg = rospy.wait_for_message("/pandaposition", PoseStamped)
                rospy.loginfo("Received at goal message!")
                endPosition = endPos(msg,rot) # transforming
                #print(endPosition)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("look up fail")    

            rospy.loginfo(endPosition)
            check = input("Enter 2 to move robot to coordinates:\n")
            check = int(check)
            if check == 2:
                captureImage("before")
                pubSuc.publish(Bool(True))
                #pub.publish(endPosition)
                moveAbove(endPosition)
                rospy.sleep(1)
                homePos()
                moveToOtherSide(endPosition)
                pubSuc.publish(Bool(False))
                rospy.sleep(1)
                homePos()
                captureImage("after")
                counter = counter+1
            else: 
                continue
        else:
            continue