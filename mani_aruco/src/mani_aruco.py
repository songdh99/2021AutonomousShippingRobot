#!/home/x/.pyenv/versions/py379/bin python
import rospy
from geometry_msgs.msg import Pose
from multi_robot.msg import aruco_msgs

rospy.init_node('aruco_pub')
aruco_xyz = rospy.Publisher('aruco_xyz', aruco_msgs, queue_size=1)
msg = aruco_msgs()
aruco_xyz.x = 1
aruco_xyz.y = 2
aruco_xyz.z = 3

while not rospy.is_shutdown():
    aruco_xyz.publish(aruco_xyz)
    

