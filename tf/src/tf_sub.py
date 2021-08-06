#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose

#mani_pose에 따른 캠에서 로봇으로 tf
def draw_cam(mani_pose):
    rospy.loginfo("tf_sub_draw_cam")
    br = tf.TransformBroadcaster()

    br.sendTransform((mani_pose.position.x, mani_pose.position.y, mani_pose.position.z),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "mani_pose",
                "/base_link")

    br.sendTransform((-0.163, 0, 0.05), (0, 0, 0, 1), rospy.Time.now(), "cam_test", "mani_pose")

#카메라 축에서 로봇 축으로 tf
def draw_rgb():
    rospy.loginfo("tf_sub_draw_rgb")
    br = tf.TransformBroadcaster()
    
    x, y, z, w = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)

    br.sendTransform((0, 0, 0),
                    (x, y, z, w),
                    rospy.Time.now(),
                    "rgb_test",
                    "cam_test")

#aruco에서 카메라로 tf
def draw_aruco(aruco_pos):
    rospy.loginfo("tf_sub_draw_aruco")
    br = tf.TransformBroadcaster()

    draw_rgb()

    # x,y,z,w = tf.transformations.quaternion_from_euler(aruco_pos.orientation.x, aruco_pos.orientation.y, aruco_pos.orientation.z)
    angle = np.sqrt(aruco_pos.orientation.x * aruco_pos.orientation.x + aruco_pos.orientation.y * aruco_pos.orientation.y + aruco_pos.orientation.z * aruco_pos.orientation.z)
    cosa = np.cos(angle * 0.5)
    sina = np.sin(angle * 0.5)
    x = aruco_pos.orientation.x * sina / angle
    y = aruco_pos.orientation.y * sina / angle
    z = aruco_pos.orientation.z * sina / angle
    w = cosa

    br.sendTransform((aruco_pos.position.x, aruco_pos.position.y, aruco_pos.position.z),
                    (x, y, z, w),
                    rospy.Time.now(),
                    "aruco_pose",
                    "rgb_test")


def main():
    rospy.init_node("tf_sub")
    rospy.Subscriber('mani_pos', Pose, draw_cam)
    rospy.Subscriber('aruco_xyzw', Pose, draw_aruco)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
