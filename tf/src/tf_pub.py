#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

aruco_check = False

#control에서 aruco 발견 여부 확인
def aruco_tf_callback(check):
    global aruco_check
    aruco_check = check.data


def main():
    rospy.init_node("tf_pub")
    global aruco_check
    pose = Pose()
    rate = rospy.Rate(10)
    listener = tf.TransformListener()
    a_about_m_pub = rospy.Publisher('a_about_m_pos', Pose, queue_size=10)
    aruco_tf_check_pub = rospy.Publisher('aruco_tf_check', Bool, queue_size=10)
    while not rospy.is_shutdown():
        rospy.Subscriber('aruco_tf_start', Bool, aruco_tf_callback)
        if aruco_check:
            try:
                (trans, rot) = listener.lookupTransform('/base_link', 'aruco_pose', rospy.Time(0))
                rospy.loginfo("tf_success")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf_except")
                continue
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            # pose.orientation.x = rot[0]
            # pose.orientation.y = rot[1]
            # pose.orientation.z = rot[2]
            # pose.orientation.w = rot[3]
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            rospy.loginfo("trans {}".format(trans))
            a_about_m_pub.publish(pose)
            aruco_tf_check_pub.publish(True)
            rospy.loginfo("aruco tf_pub end :)")
        elif not aruco_check:
            aruco_tf_check_pub.publish(False)
        rate.sleep()