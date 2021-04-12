#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, PoseArray
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs
import tf
import numpy
from tf import transformations
import tf.transformations as tr
pub = rospy.Publisher("Pointss", PointStamped, queue_size=1)

tf2Buffer = None
listener = None
#detect_count = 0
def _initialize_tf2():
    global tf2Buffer, listener
    tf2Buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf2Buffer)


def convert_pose(pose, from_frame, to_frame):

    global tf2Buffer, listener
    if tf2Buffer is None or listener is None:
        _initialize_tf2()

    try:
        trans = tf2Buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
        print(e)
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None

    spose = PoseStamped()
    spose.pose = pose
    spose.header.stamp = rospy.Time().now
    spose.header.frame_id = from_frame
    p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

    return p2
def gettf(from_frame, to_frame):

    global tf2Buffer, listener
    if tf2Buffer is None or listener is None:
        _initialize_tf2()
    try:
        trans = tf2Buffer.lookup_transform(to_frame, from_frame, rospy.Time.now(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
        #print(e)
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None
    return trans


if __name__ == "__main__":
    rospy.loginfo("gettf")
    rospy.init_node('tflistener',anonymous=True)


    rate = rospy.Rate(10.0)
    #    while not rospy.is_shutdown():
    #        try:
    #            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
    #        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #            continue
    while not rospy.is_shutdown():
        tfmat=gettf(str("camera_link"),str("shift_origin"))
        qua= (tfmat.transform.rotation.x, tfmat.transform.rotation.y, tfmat.transform.rotation.z, tfmat.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(qua)
        #print(euler)
        print(tfmat)


    rate.sleep()
    #rospy.spin()
