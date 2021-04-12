#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np                        # fundamental package for scientific computing
import time
import glob
import math
import sys
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
import roslib
roslib.load_manifest('auto_labeller')
import rospy
import math
import tf
import geometry_msgs.msg
print("init")

transCurr = []
rotCurr = []
cmdCurr = []
counter = 0
if __name__ == '__main__':
    
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    print("IN main ")
    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'robot')

    #robot_vel = rospy.Publisher('auto_labeller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    print("before while")
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_color_frame','/panda_link0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        if(counter == 0 or not(transCurr-trans==0) or not(rotCurr-rot==0)):
            transCurr = trans
            rotCurr = rot
            cmdCurr = cmd
            #robot_vel.publish(cmd)
            print("Trans", transCurr)
            print("rot", rotCurr)
            print("cmd", cmd)
        counter = counter + 1
        rate.sleep()
