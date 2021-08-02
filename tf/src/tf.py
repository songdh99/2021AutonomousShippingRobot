#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#import tf_conversions
import tf2_ros
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, TransformStamped


class Transform():
    def __init__(self):
        self.a_about_m_pub = rospy.Publisher('a_about_m_pub', Pose, queue_size=10)
        self.m_about_r_pub = rospy.Publisher('m_about_r_pub', Pose, queue_size=10)
        self.a_about_r_pub = rospy.Publisher('a_about_r_pub', Pose, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        self.st = TransformStamped()
        self.aruco_check = False
        self.mode = "draw_aruco"

    def tf_pub(self):
        if self.mode == "draw_aruco":
            rospy.loginfo_once("mode : %s", self.mode)
            rospy.Subscriber('move_aruco_pub', Bool, self.move_aruco_callback)
            if self.aruco_check:
                rospy.loginfo("a_about_m_tf")
                self.draw_rgb()
                rospy.Subscriber('aruco_xyzw', Pose, self.draw_aruco)

    def move_aruco_callback(self, check):
        self.aruco_check = check.data

    def draw_rgb(self):
        rospy.loginfo_once("draw_rgb")
        self.st.header.stamp = rospy.Time.now()
        self.st.header.frame_id = "cam_test"
        self.st.child_frame_id = "rgd_test"
        
        self.st.transform.translation.x = 0
        self.st.transform.translation.y = 0
        self.st.transform.translation.z = 0

        #x, y, z, w = tf_conversions.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
        self.st.transform.rotation.x = 0
        self.st.transform.rotation.y = 0
        self.st.transform.rotation.z = 0
        self.st.transform.rotation.w = 1
        # sendTransform(translation, quaternion, 현제간, 생성할 tf 이름, 부모 tf 이름)
        self.br.sendTransform(self.st)

    def draw_aruco(self, aruco_pos):
        rospy.loginfo_once("draw_aruco")
        self.st.header.stamp = rospy.Time.now()
        self.st.header.frame_id = "rgb_test"
        self.st.child_frame_id = "aruco_pos"

        self.st.transform.translation.x = aruco_pos.position.x
        self.st.transform.translation.y = aruco_pos.position.y
        self.st.transform.translation.z = aruco_pos.position.z

        # x,y,z,w = tf.transformations.quaternion_from_euler(aruco_pos.orientation.x, aruco_pos.orientation.y, aruco_pos.orientation.z)
        angle = np.sqrt(aruco_pos.orientation.x * aruco_pos.orientation.x + aruco_pos.orientation.y * aruco_pos.orientation.y + aruco_pos.orientation.z * aruco_pos.orientation.z)
        cosa = np.cos(angle * 0.5)
        sina = np.sin(angle * 0.5)
        self.st.transform.rotation.x = aruco_pos.orientation.x * sina / angle
        self.st.transform.rotation.y = aruco_pos.orientation.y * sina / angle
        self.st.transform.rotation.z = aruco_pos.orientation.z * sina / angle
        self.st.transform.rotation.w = cosa

        self.br.sendTransform(self.st)


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
