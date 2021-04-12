#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from geometry_msgs.msg import Quaternion

def home_pos(xcor,ycor,zcor): #home_pos(self, name):
	p = PoseStamped()
	p.header.frame_id = "/panda_suction_end" #"/panda_link8"
	p.header.stamp = rospy.Time.now()

	p.pose.position.x = xcor
	p.pose.position.y = ycor
	p.pose.position.z = zcor #0.59
	#p.pose.position.x = 0.307
	#p.pose.position.y = 0.0
	#p.pose.position.z = 0.54 #0.59

	#xyz
	p.pose.orientation.x = -0.9999489230747388
	p.pose.orientation.y = -0.006670088434895112
	p.pose.orientation.z = -0.0042415389399379526
	p.pose.orientation.w = -0.006298452933945786
        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        #self._scene.add_box(name, p, (0.5, 0.4, 0.02))

	return p 

def talker():
	pub = rospy.Publisher('keyboard', PoseStamped, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 10hz#
	counter = 0
	
	while not rospy.is_shutdown():
		value = input("Please enter an integer (1 for home, 2 for special):\n")
		value = int(value)
		if value == 1:
			position = home_pos(0.307,0.0,0.54) #x y z
			rospy.loginfo(position)
			pub.publish(position)
		elif value == 2:
			xcurr = input("X: \n")
			xcurr = float(xcurr)
			ycurr = input("Y: \n")
			ycurr = float(ycurr)
			zcurr = input("Z: \n")
			zcurr = float(zcurr)
			position = home_pos(xcurr,ycurr,zcurr) #x y z
			rospy.loginfo(position)
			pub.publish(position)
		else:
			break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


