#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

class Transform():
    def __init__(self):
        self.a_about_r_pub = rospy.Publisher('a_about_r', Pose, queue_size=10)
        self.rate = rospy.Rate(10)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.pos = Pose()
        self.aruco_check = False

    def tf_pub(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('mani_pos', Pose, self.draw_cam)
            rospy.Subscriber('aruco_tf', Bool, self.aruco_tf_callback)
            if self.aruco_check:
                rospy.loginfo("aruco tf start")
                self.draw_rgb()
                rospy.Subscriber('aruco_xyzw', Pose, self.draw_aruco)
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
                rospy.loginfo("trans {}  rot {}".format(trans, rot))
                self.a_about_r_pub.publish(self.pos)
                rospy.sleep()
                rospy.loginfo("aruco tf end")
                self.aruco_check = False

    #mani_pos에 따른 캠에서 로봇으로 tf
    def draw_cam(self, mani_pose):
        rospy.loginfo("draw_cam")
        mani_pos = np.array([[mani_pose.position.x, mani_pose.position.y, mani_pose.position.z]])
        mani_ori = [mani_pose.orientation.x, mani_pose.orientation.y, mani_pose.orientation.z, mani_pose.orientation.w]
        r, p, y = tf.transformations.euler_from_quaternion(mani_ori)
        mani_3d_matrix = make_3d_matrix(mani_pos, mani_ori)
        cam_rgb_pos = np.array([[-0.06, 0, 0.04]])
        cam_rgb_rpy = [0, 0, 0]
        cam_rotation = create_rotation_matrix(cam_rgb_rpy)
        cam_3d_matrix = make_transformation_matrix(cam_rotation, cam_rgb_pos.T)
        m_c_matrix = np.dot(mani_3d_matrix, cam_3d_matrix)
        t, q = get_tq_to_transformation_matrix(m_c_matrix)
        self.br.sendTransform(t, q, rospy.Time.now(), "cam_test", "base_link")
        self.br.sendTransform((mani_pose.position.x, mani_pose.position.y, mani_pose.position.z),
                     (mani_pose.orientation.x, mani_pose.orientation.y, mani_pose.orientation.z, mani_pose.orientation.w),
                     rospy.Time.now(),
                     "mani_pose",
                     "base_link")

    #control에서 aruco 발견 여부 확인
    def aruco_tf_callback(self, check):
        self.aruco_check = check.data

    #카메라 축에서 로봇 축으로 tf
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
        rospy.loginfo("draw_aruco")

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

#3d transformation matrix 생성
def make_3d_matrix(position, orientation):
    euler = euler_from_quaternion(orientation)
    rotation = create_rotation_matrix(euler)
    matrix_3d = make_transformation_matrix(rotation, position.T)
    return matrix_3d

#쿼터니언을 오일러로 변환
def euler_from_quaternion(orientation):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation)
    euler = [roll, pitch, yaw]
    return euler

#오일러를 쿼터니언으로 변환
def quaternion_from_euler(yaw, pitch, roll):
    x, y, z, w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    quaternion = [x, y, z, w]
    return quaternion

#rotation matrix 생성
def create_rotation_matrix(euler):
    (roll, pitch, yaw) = euler

    yaw_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    roll_matrix = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    r_dot_p = np.dot(roll_matrix, pitch_matrix)
    rotation_matrix = np.dot(yaw_matrix, r_dot_p)

    return rotation_matrix

#transformation matrix 생성
def make_transformation_matrix(r_matrix, t_matrix):
    #행렬의 열을 합침
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    #행렬의 행을 합침
    transformation_matrix = np.concatenate((matrix_3x4, zero_one), axis=0)
    return transformation_matrix

#transformation matrix에서 translation과 quaternion 추출
def get_tq_to_transformation_matrix(matrix):
    translation = list(matrix[:3, 3])

    r_11 = matrix[0, 0]  # cos(yaw)cos(pitch)
    r_21 = matrix[1, 0]  # sin(yaw)cos(pitch)
    r_31 = matrix[2, 0]  # -sin(pitch)
    r_32 = matrix[2, 1]  # cos(pitch)sin(roll)
    r_33 = matrix[2, 2]  # cos(pitch)cos(roll)
    yaw = np.arctan2(r_21, r_11)
    pitch = np.arctan2(-r_31, np.sqrt((np.square(r_32)) + np.square(r_33)))
    roll = np.arctan2(r_32, r_33)
    quaternion = quaternion_from_euler(yaw, pitch, roll)

    return translation, quaternion


def main():
    rospy.init_node("tf")
    tran = Transform()
    tran.tf_pub()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
