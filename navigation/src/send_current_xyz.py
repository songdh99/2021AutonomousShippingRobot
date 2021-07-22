#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
import tf

def main():
	rospy.init_node('send_current_xyz')
	listener = tf.TransformListener()
	pub = rospy.Publisher('current_xyz', Pose, queue_size=1)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		start_xyz = Pose()
		start_xyz.position.x = trans[0]
		start_xyz.position.y = trans[1]
		start_xyz.position.z = trans[2]

		rospy.loginfo(start_xyz)
		pub.publish(start_xyz)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
