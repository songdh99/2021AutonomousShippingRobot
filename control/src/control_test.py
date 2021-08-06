#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Bool, String

class Tower():
	def __init__(self):
		self.DWA_pub = rospy.Publisher('DWA', String, queue_size=10)
		self.aruco_tf_pub = rospy.Publisher('aruco_tf_start', String, queue_size=10)
		self.mani_pub = rospy.Publisher('pick_or_place_pub', String, queue_size=10)
		self.mode = "patrol"
		self.stop_check = False
		self.find_aruco_check = False
		self.aruco_tf_check = False
		self.pick_check = False
		self.place_check = False
		self.mani_error = False
		

	def tower(self):
		#목표물을 향해 돌아다니기
		if self.mode == "patrol":
			rospy.loginfo("mode : %s", self.mode)
			self.DWA_pub.publish("patrol")
			self.mode = "find_aruco"
			rospy.loginfo("mode : %s", self.mode)

		#aruco를 발견하면 stop
		if self.mode == "find_aruco":
			rospy.Subscriber('check_aruco', Bool, self.find_aruco_check_callback)
			if self.find_aruco_check:
				rospy.loginfo("Find aruco marker :)")
				self.DWA_pub.publish("stop")
				self.aruco_tf_pub.publish("tf_for_DWA")
				self.mode = "tf_for_DWA"
				rospy.loginfo("mode : %s", self.mode)

		#aruco에 대한 tf가 끝났는지 확인
		if self.mode == "tf_for_DWA":
			rospy.Subscriber('aruco_tf_check', Bool, self.aruco_tf_check_callback)
			if self.aruco_tf_check:
				rospy.loginfo("Finish aruco tf :)")
				self.aruco_tf_pub.publish("Wait")
				self.mode = "move_aruco"

		#robot이 aruco로 이동
		if self.mode == "move_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.DWA_pub.publish("move_aruco")
			self.mode = "wait_goal"
			rospy.loginfo("mode : %s", self.mode)

		#robot이 도착했는지 확인
		if self.mode == "wait_goal":
			rospy.Subscriber('stop_point', String, self.stop_callback)
			if self.stop_check:
				rospy.loginfo("Land goal :)")
				self.DWA_pub.publish("stop")
				self.mode = "wait_aruco"
				rospy.loginfo("mode : %s", self.mode)

		#aruco 찾은 후 tf 변환
		if self.mode == "wait_aruco":
			rospy.Subscriber('check_aruco', Bool, self.find_aruco_check_callback)
			if self.find_aruco_check:
				rospy.loginfo("Find aruco marker :)")
				self.aruco_tf_pub.publish("tf_for_mani")
				self.mode = "tf_for_mani"
				rospy.loginfo("mode : %s", self.mode)

		#aruco에 대한 tf가 끝났는지 확인
		if self.mode == "tf_for_mani":
			rospy.Subscriber('aruco_tf_check', Bool, self.aruco_tf_check_callback)
			if self.aruco_tf_check:
				rospy.loginfo("Finish aruco tf :)")
				self.aruco_tf_pub.publish("Wait")
				self.mode = "pick_aruco"

		#aruco로 팔 이동
		if self.mode == "pick_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.mani_pub.publish("pick")
			self.mode = "pick_aruco_check"
			rospy.loginfo("mode : %s", self.mode)

		#aruco를 잡았는지 확인
		if self.mode == "pick_aruco_check":
			rospy.Subscriber('fin_call_pub', Bool, self.pick_check_callback)
			if self.pick_check:
				rospy.loginfo("Pick aruco marker :)")
				self.mani_pub.publish("Wait")
				self.DWA_pub.publish("home")
				self.mode = "wait_home"
				rospy.loginfo("mode : %s", self.mode)

		#귀환
		if self.mode == "wait_home":
			rospy.Subscriber('stop_point', String, self.stop_callback)
			if self.stop_check:
				rospy.loginfo("Land home :)")
				self.DWA_pub.publish("stop")
				self.mode = "place_aruco"
				rospy.loginfo("mode : %s", self.mode)

		#aruco를 home에 드랍
		if self.mode == "place_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.mani_pub.publish("place")
			self.mode = "place_aruco_check"

		#aruco를 잡았는지 확인
		if self.mode == "place_aruco_check":
			rospy.loginfo("mode : %s", self.mode)
			rospy.Subscriber('fin_call_pub', Bool, self.place_check_callback)
			if self.place_check:
				rospy.loginfo("Place aruco marker :)")
				self.mani_pub.publish("Wait")
				self.mode = "patrol"


	def stop_callback(self, check):
		if(check.data == "stop"):
			self.stop_check = True
		else:
			self.stop_check = False

	def find_aruco_check_callback(self, check):
		self.find_aruco_check = check.data

	def aruco_tf_check_callback(self, check):
		self.aruco_tf_check = check.data

	def pick_check_callback(self, check):
		self.pick_check = check.data

	def place_check_callback(self, check):
		self.place_check = check.data
			
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
