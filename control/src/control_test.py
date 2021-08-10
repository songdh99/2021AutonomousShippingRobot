#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Bool, String

class Tower():
	def __init__(self):
		self.DWA_pub = rospy.Publisher('DWA_pub', String, queue_size=10)
		self.aruco_tf_pub = rospy.Publisher('aruco_tf_start', Bool, queue_size=10)
		self.mani_pub = rospy.Publisher('pick_or_place_pub', String, queue_size=10)

		self.mode = "patrol"
		self.stop_check = False
		self.find_aruco_check = False
		self.aruco_tf_check = False
		self.pick_check = False
		self.place_check = False
		self.mani_error = False
		self.count = 0
		

	def tower(self):
		#목표물을 향해 돌아다니기
		if self.mode == "patrol":
			rospy.loginfo("mode : %s", self.mode)
			self.stop_check = False
			self.place_check = False
			self.mode = "wait_goal"
			rospy.loginfo("mode : %s", self.mode)

		#robot이 도착했는지 확인
		if self.mode == "wait_goal":
			self.DWA_pub.publish("patrol")
			rospy.Subscriber('stop_point', String, self.stop_callback)
			if self.stop_check:
				rospy.loginfo("Land goal :)")
				self.mode = "find_aruco"
				rospy.loginfo("mode : %s", self.mode)

		#aruco 찾은 후 tf 변환
		if self.mode == "find_aruco":
			self.DWA_pub.publish("stop")
			self.stop_check = False
			self.mani_error = False
			self.count = 0
			rospy.Subscriber('check_aruco', Bool, self.find_aruco_check_callback)
			if self.find_aruco_check:
				rospy.loginfo("Find aruco marker :)")
				self.aruco_tf_pub.publish(True)
				self.mode = "tf_for_mani"
				rospy.loginfo("mode : %s", self.mode)
			else:
				rospy.loginfo("Couldn't find aruco marker")
				self.aruco_tf_pub.publish(False)

		#aruco에 대한 tf가 끝났는지 확인
		if self.mode == "tf_for_mani":
			self.find_aruco_check = False
			rospy.Subscriber('aruco_tf_check', Bool, self.aruco_tf_check_callback)
			if self.aruco_tf_check:
				rospy.loginfo("Finish aruco tf :)")
				self.aruco_tf_pub.publish(False)
				self.mode = "pick_aruco"

		#aruco로 팔 이동
		if self.mode == "pick_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.aruco_tf_check = False
			self.mani_pub.publish("pick")
			self.mode = "fin_mani_check"
			rospy.loginfo("mode : %s", self.mode)

		#mani finish check
		if self.mode == "fin_mani_check":
			rospy.Subscriber('fin_call_pub', Bool, self.fin_check_callback)
			if self.pick_check:
				rospy.loginfo("Finish mani :)")
				self.mani_pub.publish("Wait")
				self.mode = "pick_aruco_check"
				rospy.loginfo("mode : %s", self.mode)

		#aruco를 잡았는지 확인
		if self.mode == "pick_aruco_check":
			self.pick_check = False
			rospy.Subscriber('mani_error', Bool, self.pick_check_callback)
			if self.mani_error:
				rospy.loginfo("Didn't pick aruco marker :(")
				self.mode = "find_aruco"
				rospy.loginfo("mode : %s", self.mode)
			else:
				self.count += 1
				rospy.loginfo("{}".format(self.count))
				if self.count == 100:
					rospy.loginfo("Pick aruco marker :)")
					self.mode = "wait_home"
					rospy.loginfo("mode : %s", self.mode)

		#귀환
		if self.mode == "wait_home":
			self.mani_error = False
			self.DWA_pub.publish("home")
			rospy.Subscriber('stop_point', String, self.stop_callback)
			if self.stop_check:
				rospy.loginfo("Land home :)")
				self.mode = "place_aruco"
				rospy.loginfo("mode : %s", self.mode)

		#aruco를 home에 드랍
		if self.mode == "place_aruco":
			rospy.loginfo("mode : %s", self.mode)
			self.stop_check = False
			self.mode = "place_aruco_check"

		#aruco를 잡았는지 확인
		if self.mode == "place_aruco_check":
			rospy.loginfo("mode : %s", self.mode)
			self.mani_pub.publish("place")
			self.DWA_pub.publish("stop")
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

	def fin_check_callback(self, check):
		self.pick_check = check.data

	def pick_check_callback(self, check):
		self.mani_error = check.data

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
