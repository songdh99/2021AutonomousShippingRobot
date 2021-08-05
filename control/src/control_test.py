#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose

class Tower():
	def __init__(self):
		self.DWA_pub = rospy.Publisher('DWA', String, queue_size=10)
		self.aruco_tf_pub = rospy.Publisher('aruco_tf', Bool, queue_size=10)
		self.mani_pub = rospy.Publisher('pick_or_place_pub', String, queue_size=10)
		self.mode = "patrol"
		self.stop_check = False
		self.aruco_check = False
		self.pick_check = False
		self.mani_error = False
		

	def tower(self):
		#목표물을 향해 돌아다니기
		if self.mode == "patrol":
			rospy.loginfo("mode : %s", self.mode)
			self.DWA_pub.publish("patrol")
			self.mode = "wait_goal"

		#목표지점에 도착했는지 확인
		if self.mode == "wait_goal":
			rospy.loginfo("mode : %s", self.mode)
			#rospy.Subscriber('stop_point', String, self.stop_callback)
			#if self.stop_check:
			#	self.mode = "find_aruco"
			self.mode = "find_aruco"

		#aruco 찾은 후 tf 변환
		if self.mode == "find_aruco":
			rospy.loginfo_once("mode : %s", self.mode)
			rospy.Subscriber('check_aruco', Bool, self.aruco_check_callback)
			if self.aruco_check:
				rospy.loginfo("Find aruco marker")
				self.aruco_tf_pub.publish(True)
				self.mode = "move_aruco"
			elif not self.aruco_check:
				self.aruco_tf_pub.publish(False)

		#aruco로 팔 이동
		if self.mode == "move_aruco":
			rospy.loginfo_once("mode : %s", self.mode)
			self.mani_pub.publish("pick")
			#publish(aruco 좌표)
			rospy.Subscriber('fin_call_pub', Bool, self.pick_check_callback)
			if self.pick_check:
				rospy.loginfo("Pick aruco marker")
				self.DWA_pub.publish("home")


	def stop_callback(self, check):
		if(check.data == "stop"):
			self.stop_check = True
		else:
			self.stop_check = False

	def aruco_check_callback(self, check):
		self.aruco_check = check.data

	def pick_check_callback(self, check):
		self.pick_check = check.data
			
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
