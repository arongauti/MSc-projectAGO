#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, PoseArray
from vision_msgs.msg import OBB as obb
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs
import tf
import numpy
from tf import transformations
import tf.transformations as tr
pub = rospy.Publisher("Pointss", PointStamped, queue_size=1)
pub2 = rospy.Publisher("poses", PoseStamped, queue_size=1)
pub3 = rospy.Publisher("arr", PoseArray, queue_size=10)
obb_pub = rospy.Publisher("obb", obb, queue_size=10)
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

class CameraPosition():

    def __init__(self):
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tags)
        r1=None
        r2=None
        self.arr_pose = PoseArray()
        self.arr_pose.header.frame_id = 'panda_link0'
        self.arr_pose.header.stamp = rospy.Time.now()
        self.arr_pose.poses = []
        real=True
        self.lowleft=13
        self.lowright=14
        self.upleft=11
        self.upright=12
        real = rospy.get_param("/scan/real")
        print("-------------------TRUE?---")
        print(real)
        if real:
            self.lowleft = 8
            self.lowright = 9
            self.upleft = 0
            self.upright = 1
            print("-------------------TRUE?---")
            print(real)

        #self.history = []

    #def movingavg(self,centroids):
    #    global history
    #    history.append(msg.data)
    #    if len(history) > 60:
    #        history = history[-60:]
    #    average = sum(history) / float(len(history))
    #    rospy.loginfo('Average of most recent {} samples: {}'.format(len(history), average))
    def posearr_comb(self,centerposes):
        global detect_count
        detect_count+=1
        #print(detect_count)
        #if detect_count < 100:
        #    self.arr_pose.poses.append(centerposes)
        #else:
        #    pub3.publish(self.arr_pose)
        #    print("now")
        #if len(self.arr_pose.poses)>60:
        #    self.arr_pose.poses=self.arr_pose.poses[-60:]
        #avg = sum
        #rospy.loginfo('Average of most recent {} samples: {}'.format(len(history), average))






    def get_tags(self, msg):

        #pos_x = msg.detections[0].pose.pose.pose.position.x
        #print(msg.detections[0].id)
        #print("number of detections:",len(msg.detections))
        ids=len(msg.detections)
        ul_pose=PoseStamped()
        ur_pose = PoseStamped()
        ll_pose = PoseStamped()
        lr_pose = PoseStamped()
        #p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)
        tf_mat=gettf(msg.detections[0].pose.header.frame_id,str("panda_link0"))
        if ids==4:

            for detection in msg.detections:
                #print(detection.id)
                if detection.id[0]==self.upleft:
                    #print(detection.id)
                    #print('upperleft')
                    ul_pose.header.stamp = tf_mat.header.stamp
                    ul_pose.header.frame_id = msg.detections[0].pose.header.frame_id
                    ul_pose.pose=detection.pose.pose.pose
                    ul_pose = tf2_geometry_msgs.do_transform_pose(ul_pose,tf_mat)
                    #print("upperleft")
                    #print(ul_pose)
                    #print(p2)
                    #upperleft=detection.pose.pose.pose.position
                    upperleft = ul_pose.pose.position
                elif detection.id[0]==self.upright:
                    #print(detection.id)
                    #print('upperright')
                    ur_pose.header.stamp = tf_mat.header.stamp
                    ur_pose.header.frame_id = msg.detections[0].pose.header.frame_id
                    ur_pose.pose=detection.pose.pose.pose
                    ur_pose = tf2_geometry_msgs.do_transform_pose(ur_pose,tf_mat)
                    upperright=ur_pose.pose.position
                    #upperright=detection.pose.pose.pose.position

                elif detection.id[0]==self.lowleft:
                    #print(detection.id)
                    #print("lowerleft")
                    ll_pose.header.stamp = tf_mat.header.stamp
                    ll_pose.header.frame_id = msg.detections[0].pose.header.frame_id
                    ll_pose.pose=detection.pose.pose.pose
                    ll_pose = tf2_geometry_msgs.do_transform_pose(ll_pose,tf_mat)
                    lowerleft=ll_pose.pose.position
                    #lowerleft=detection.pose.pose.pose.position

                elif detection.id[0]==self.lowright:
                    lr_pose.header.stamp = tf_mat.header.stamp
                    lr_pose.header.frame_id = msg.detections[0].pose.header.frame_id
                    lr_pose.pose=detection.pose.pose.pose
                    lr_pose = tf2_geometry_msgs.do_transform_pose(lr_pose,tf_mat)
                    #print(detection.id)
                    #print('lowerright')
                    lowerright = lr_pose.pose.position
                    #lowerright=detection.pose.pose.pose.position
            x_ax=numpy.array([ul_pose.pose.position.x-ll_pose.pose.position.x,ul_pose.pose.position.y-ll_pose.pose.position.y,
                              ul_pose.pose.position.z-ll_pose.pose.position.z])
            y_ax = numpy.array([ll_pose.pose.position.x - lr_pose.pose.position.x, ll_pose.pose.position.y - lr_pose.pose.position.y,
                                ll_pose.pose.position.z - lr_pose.pose.position.z])
            boxlen = tr.vector_norm(x_ax) #x distance
            boxwidth = tr.vector_norm(x_ax) #ydist

            x_ax=tr.unit_vector(x_ax)
            y_ax=tr.unit_vector(y_ax)
            z_ax= numpy.cross(x_ax,y_ax)
            #rmat=tr.identity_matrix()
            Rx=tr.rotation_matrix(0,x_ax)
            Ry = tr.rotation_matrix(0, y_ax)
            Rz = tr.rotation_matrix(0, z_ax)
            rmat=tr.concatenate_matrices(Rx,Ry,Rz)
            aaa=tr.vector_norm(x_ax)
            centerquat=tr.quaternion_from_matrix(rmat)
            #print(rqq)
            #rmat=numpy.asarray(rmat)
            #print(rmat)
            #R_map_lastbaselink = tr.quaternion_matrix(q_map_lastbaselink)[0:3, 0:3]
            #p_lastbaselink_currbaselink = R_map_lastbaselink.transpose().dot(p_map_currbaselink - p_map_lastbaselink)
            cx=(lowerleft.x+lowerright.x+upperleft.x+upperright.x)/4.0
            cy=(lowerleft.y+lowerright.y+upperleft.y+upperright.y)/4.0
            cz=(lowerleft.z+lowerright.z+upperleft.z+upperright.z)/4.0
            centerpose=Pose()
            centpose=PoseStamped()
            #centerpose.orientation=msg.detections[0].pose.pose.pose.orientation
            centpose.pose.position.x=cx
            centpose.pose.position.y = cy
            centpose.pose.position.z = cz
            centpose.pose.orientation.x=centerquat[0]
            centpose.pose.orientation.y = centerquat[1]
            centpose.pose.orientation.z = centerquat[2]
            centpose.pose.orientation.w = centerquat[3]
            centpose.header.frame_id=lr_pose.header.frame_id
            centpose.header.stamp=lr_pose.header.stamp
            #print("a:\n",centerpose)
            centerpose=centpose.pose
            bbox=obb()
            bbox.width=boxwidth
            bbox.height=boxlen
            bbox.depth=cz #pt zero
            bbox.pose_orient=centpose
            obb_pub.publish(bbox)
            #centersp=transformedp.pose
            #print(transformedp)
            #q_map_baselink = numpy.array([q.x, q.y, q.z, q.w])
            #qua1=msg.detections[0].pose.pose.pose.orientation
            #qua2=msg.detections[1].pose.pose.pose.orientation
            #qua3=msg.detections[2].pose.pose.pose.orientation
            #qua4=msg.detections[3].pose.pose.pose.orientation
            #qq1=numpy.array([qua1.x,qua1.y,qua1.z,qua1.w])
            #qq2=numpy.array([qua2.x,qua2.y,qua2.z,qua2.w])
            #qq3=numpy.array([qua3.x,qua3.y,qua3.z,qua3.w])
            #qq4=numpy.array([qua4.x,qua4.y,qua4.z,qua4.w])
            #r1=transformations.quaternion_matrix(qq1)
            #r2=transformations.quaternion_matrix(qq2)
            #r3=transformations.quaternion_matrix(qq3)
            #r4=transformations.quaternion_matrix(qq4)
            #q_last_to_current = tr.quaternion_multiply(tr.quaternion_inverse(qq1), qq2)
            #r_d, p_d, yaw_diff = tr.euler_from_quaternion(q_last_to_current)

            pub2.publish(centpose)
        elif ids==2:
            for detection in msg.detections:
                #print(detection.id)
                if detection.id[0]==13 or detection.id[0]==14:
                    print("lower 2 detected, move forward")



if __name__ == "__main__":
    rospy.loginfo("transform extrinsics")
    rospy.init_node('apriltag_calib',anonymous=True)
    detect_count=0
    CameraPosition()
    rospy.spin()
