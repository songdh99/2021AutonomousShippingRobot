#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool


def main():
	rospy.init_node('send_startpos')
	listener = tf.TransformListener()
	turtle_vel = rospy.Publisher('send_startpos_Pub', Pose, queue_size=1)	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		nav_goal = Pose()
		
		nav_goal.position.x = trans[0]
		nav_goal.position.y = trans[1]
		nav_goal.position.z = trans[2]		
		
		print("x: {}  y:{}  z:{}".format(trans[0], trans[1], trans[2]))
		turtle_vel.publish(nav_goal)

if__name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
