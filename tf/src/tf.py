#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


class Transform():
    def __init__(self):
        self.a_about_r_pub = rospy.Publisher('a_about_r', Pose, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.pos = Pose()
        self.aruco_check = False
        self.mode = "m_about_r"

    def tf_pub(self):
        if self.mode == "a_about_c":
            rospy.Subscriber('aruco_tf', Bool, self.aruco_tf_callback)
            if self.aruco_check:
                rospy.loginfo("mode : %s", self.mode)
                self.draw_rgb()
                rospy.Subscriber('aruco_xyzw', Pose, self.draw_aruco)
                self.mode = "a_about_r"

        if self.mode == "a_about_r":
            rospy.loginfo("mode : %s", self.mode)
            while self.mode == "a_about_r":
                try:
                    (trans, rot) = self.listener.lookupTransform('/base_link', 'aruco_pos', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                self.pos.position.x = trans[0]
                self.pos.position.y = trans[1]
                self.pos.position.z = trans[2]
                self.pos.orientation.x = rot[0]
                self.pos.orientation.y = rot[1]
                self.pos.orientation.z = rot[2]
                self.pos.orientation.w = rot[3]
                self.a_about_r_pub.publish(self.pos)
                self.mode = "a_about_c"

    #control에서 aruco 발견 여부 확인
    def move_aruco_callback(self, check):
        self.aruco_check = check.data

    #카메라 축에서 로봇 축으로 변환
    def draw_rgb(self):
        rospy.loginfo("draw_rgb")

        x, y, z, w = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)

        # sendTransform(translation, quaternion, 현재시간, 생성할 tf 이름, 부모 tf 이름)
        self.br.sendTransform((0, 0, 0),
                         (x, y, z, w),
                         rospy.Time.now(),
                         "rgb_test",
                         "cam_test")

    #aruco에서 카메라로 tf
    def draw_aruco(self, aruco_pos):
        rospy.loginfo_once("draw_aruco")

        # x,y,z,w = tf.transformations.quaternion_from_euler(aruco_pos.orientation.x, aruco_pos.orientation.y, aruco_pos.orientation.z)
        angle = np.sqrt(aruco_pos.orientation.x * aruco_pos.orientation.x + aruco_pos.orientation.y * aruco_pos.orientation.y + aruco_pos.orientation.z * aruco_pos.orientation.z)
        cosa = np.cos(angle * 0.5)
        sina = np.sin(angle * 0.5)
        x = aruco_pos.orientation.x * sina / angle
        y = aruco_pos.orientation.y * sina / angle
        z = aruco_pos.orientation.z * sina / angle
        w = cosa

        self.br.sendTransform((aruco_pos.position.x, aruco_pos.position.y, aruco_pos.position.z),
                     (x, y, z, w),
                     rospy.Time.now(),
                     "aruco_pos",
                     "rgb_test")


def main():
    rospy.init_node("tf")
    tran = Transform()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tran.tf_pub()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
