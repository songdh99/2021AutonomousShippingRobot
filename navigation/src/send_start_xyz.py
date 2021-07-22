#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

def main():
	pub = rospy.Publisher('start_xyz', Pose, queue_size=1)
	rospy.init_node('send_start_xyz')
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

		start_xyz = Pose()
		start_xyz.position.x = 1
		start_xyz.position.y = 1
		start_xyz.position.z = 1
		rospy.loginfo(start_xyz)
		pub.publish(start_xyz)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
