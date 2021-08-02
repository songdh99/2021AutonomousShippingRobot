#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose

class Tower():
	def __init__(self):
		self.DWA_pub = rospy.Publisher('DWA_pub', Bool, queue_size=10)
		self.mani_pub = rospy.Publisher('mani_pub', Bool, queue_size=10)
		self.move_aruco_pub = rospy.Publisher('move_aruco_pub', Bool, queue_size=10)
		self.mode = "patrol"
		#mani가 목표물을 잡았는지 판단
		self.mani = False
		self.aruco_check = False
		
	def tower(self):
	#목표물을 향해 돌아다니기
		if self.mode == "patrol":
			rospy.loginfo("mode : %s", self.mode)
			self.DWA_pub.publish(True)
			self.mode = "find_aruco"
			
		#aruco 찾기
		if self.mode == "find_aruco":
			rospy.loginfo_once("mode : %s", self.mode)
			rospy.Subscriber('check_aruco', Bool, self.aruco_check_callback)
			if self.aruco_check:
				rospy.loginfo("Find aruco marker")
				self.move_aruco_pub.publish(True)
				#self.mode = "move_aruco"
			elif not self.aruco_check:
    				self.move_aruco_pub.publish(False)

		#aruco로 팔 이동
		if self.mode == "move_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.mani_pub.publish(True)

	def aruco_check_callback(self, check):
		self.aruco_check = check.data
			
def main():
    rospy.init_node("contol_tower")
    control = Tower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        control.tower()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
