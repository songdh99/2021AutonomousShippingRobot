#!/home/x/.pyenv/versions/py379/bin python
import rospy
from geometry_msgs.msg import Pose

def main():
	rospy.init_node('aruco_pub')
	aruco_xyz = rospy.Publisher('aruco_xyz', Pose, queue_size=1)
	msg = Pose()
	msg.position.x = 0.1
	msg.position.y = 0.1
	msg.position.z = 0.1
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		aruco_xyz.publish(msg)
		rate.sleep()
	
	    

