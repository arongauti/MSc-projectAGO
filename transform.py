#!/usr/bin/env python  
import roslib
roslib.load_manifest('auto_labeller')
import rospy
import math
import tf
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from geometry_msgs.msg import Quaternion

error = 0.05 # error from the camera, since it is tilted
suctionCup = 0.056 # length from endeffector to end of suction cup

def endPos(msg,rot): 
    """ 
    (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
     """# Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    #Find difference between camera and suction end
    diff=PointStamped()
    diff.header.frame_id = str(msg.header.frame_id) #camera_color_optical_frame
    diff.header.stamp = rospy.Time(0)
    diff.point.x=position.x  #z
    diff.point.y=position.y #x
    diff.point.z=position.z #-error #y 
    p=listener.transformPoint("panda_suction_end",diff)

    pickPoint=PointStamped()
    pickPoint.header.frame_id = "panda_suction_end"
    pickPoint.header.stamp = rospy.Time(0)
    pickPoint.point.x=p.point.x
    pickPoint.point.y=p.point.y
    pickPoint.point.z=p.point.z-suctionCup
    p = listener.transformPoint("panda_link0",pickPoint)
    
    pe = PoseStamped()
    pe.header.frame_id = "/panda_link0"
    pe.header.stamp = rospy.Time.now()
    pe.pose.orientation.x = rot[0]
    pe.pose.orientation.y = rot[1]
    pe.pose.orientation.z = rot[2]
    pe.pose.orientation.w = rot[3]
    pe.pose.position.x = p.point.x
    pe.pose.position.y = p.point.y
    pe.pose.position.z = p.point.z
    return pe     

def callback(msg):
    
    rospy.loginfo("Received at goal message!")
    rospy.loginfo("Timestamp: " + str(msg.header.stamp))
    rospy.loginfo("frame_id: " + str(msg.header.frame_id))
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/panda_link0','/panda_suction_end', rospy.Time(0))
            print(trans)
            endPosition = endPos(msg,rot)
            #print(endPosition)
            value = input("Enter 1 to get coordinates:\n")
            value = int(value)
            if value == 1:
                rospy.loginfo(endPosition)
                check = input("Enter 2 to move robot to coordinates:\n")
                check = int(check)
                if check == 2:
                    pub.publish(endPosition)
                else: 
                    break
            else: 
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("look up fail")
            continue

if __name__ == '__main__':
    pub = rospy.Publisher('keyboard', PoseStamped, queue_size=10)
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rospy.Subscriber("/pandaposition", PoseStamped, callback)
    rospy.spin()