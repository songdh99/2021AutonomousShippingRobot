#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, PoseStamped
from sensor_msgs.msg import LaserScan

# 속도, 각속도의 개수
mps_c = 2
rps_c = 13

Mps = [0.15, 0.13]
Radps = [0, 0.3, -0.3, 0.5, -0.5, 0.6, -0.6, 0.7, -0.7, 0.8, -0.8, 0.9, -0.9]  # 첫 원소는 무조건 0을 넣어야 함 (계산식이 다르기 때문)


five_Radps_scandistance = np.full((10, 1, rps_c), 0.)  # 10스텝까지의 다섯개의 각속도에 따른 각도마다 스캔값 저장
# 속도, 각속도에 따라 도달하는 직선거리값을 step 마다 계산
# 한번 계산하고 계속 사용하기 위해 함수 밖에 작성
MpsAr = np.array(Mps).reshape(mps_c, 1)
RadpsAr = np.delete(np.array(Radps), 0)  # 각속도가 0일땐 거리계산식이 달라지므로 제외 후 따로 계산
step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)
zeroRadpsAr = MpsAr * step  # 각속도가 0일때 (10, mps_c, 1)
distancestep = (2 * np.sin(RadpsAr * step / 2) / RadpsAr * MpsAr)  # (10, mps_c, rps_c-1)
fulldistancesteps = np.concatenate((zeroRadpsAr, distancestep), axis=2) + 0.3  # (10, mps_c, rps_c) 로봇의 크기보정을 위해 + 0.2

angle160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
dg_angle160_Radps_step = np.int32(
    np.rint(angle160 + np.degrees(step * np.array(Radps))))  # (160, 10, 1, rps_c) 반올림 후 정수형으로 변환

# (Local)로봇 기준 이동시 x, y 이동거리 (10, mps_c, rps_c)
x_move_distance = np.concatenate((zeroRadpsAr, (distancestep * np.cos(90 - (180 - step * RadpsAr) / 2))), axis=2)
y_move_distance = np.concatenate((np.zeros((10, mps_c, 1)), (distancestep * np.sin(90 - (180 - step * RadpsAr) / 2))),
                                 axis=2)
# 좌표계 변환을 위해 x,y의 9스텝 까지의 이동거리를 합쳐서 (rps_c, 2)로 만듦
xy_move_distance = np.concatenate(
    (np.reshape((x_move_distance), (10, -1, 1)), np.reshape((y_move_distance), (10, -1, 1))), axis=2)

SCAN_ran = np.full((1, 360), 0)  # 360도 측정 거리값 초기화
current_xyz = Pose()
current_angle = Pose()
stop_point = String()
wherestop = String()
goal_location_x = 0.4283
goal_location_y = -1.6591
start_location_x = 0.
start_location_y = 0.
goal_radian = 0.
retry = 0
r_g_score = np.arange(0, rps_c)
near_dis_score = np.arange(0, rps_c)
o = 0
n = 0
DWA_mode = String()
R_G_dis = 0.


class SelfDrive:

    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_point = rospy.Publisher('stop_point', String, queue_size=1)
        rospy.Subscriber('DWA_pub', String, self.check_mode)
        rospy.Subscriber('current_xyz', Pose, self.current_xyz)
        rospy.Subscriber('current_angle', Pose, self.current_angle)

    def current_angle(self, angle):
        global current_angle
        current_angle.position.z = angle.position.z * 180 / math.pi  # angle.position.z가 360도가 1로 나타남
        if current_angle.position.z < 0:
            current_angle.position.z += 360

    def current_xyz(self, xyz):
        global current_xyz
        global retry
        global start_location_x
        global start_location_y
        global wherestop
        current_xyz.position.x = xyz.position.x
        current_xyz.position.y = xyz.position.y
        retry += 1
        if retry == 1:
            wherestop = "goal point"
            start_location_x = current_xyz.position.x
            start_location_y = current_xyz.position.y

    def check_mode(self, DWA_pub):
        global o
        global DWA_mode
        global wherestop
        DWA_mode = DWA_pub.data  ###########고쳐야됨
        if DWA_mode == "home" and o == 0:
            wherestop = "starting point"
            o = 1

    def near_dis_score(self):
        global near_dis_score

        # (160, 10, 1, rps_c) 각도를 스캔한 거리값으로 변경
        a_R_s_scandistance = np.where(True, SCAN_ran[dg_angle160_Radps_step], SCAN_ran[dg_angle160_Radps_step])
        # (160, 10, mps_c, rps_c)
        neardis160 = np.sqrt((a_R_s_scandistance * np.sin(np.radians(dg_angle160_Radps_step))) ** 2 + (
                fulldistancesteps - a_R_s_scandistance * abs(np.cos(np.radians(dg_angle160_Radps_step)))) ** 2)
        # (10, mps_c, rps_c)
        near_dis = np.amin(neardis160, axis=0)
        near_dis_5 = near_dis[5]
        near_dis_before = np.where(near_dis_5 > 0.30, 0.5, near_dis_5)  # 30cm가 넘는 것은 30cm로 만듦
        near_dis_score = np.where(near_dis_before < 0.15, -100, near_dis_before)  # 10cm 보다 낮은 것은 -1로 만듦


    def goal_robot_dis(self):
        global retry
        global goal_radian
        global wherestop
        global R_G_dis
        global r_g_score

        x = goal_location_x - current_xyz.position.x
        y = goal_location_y - current_xyz.position.y
        goal_radian = math.atan2(y, x) * 180 / math.pi
        if goal_radian < 0:
            goal_radian += 360
        R_G_dis = np.hypot(goal_location_x - current_xyz.position.x, goal_location_y - current_xyz.position.y)

        if R_G_dis < 0.30 and wherestop == "goal point":
            wherestop = "stop_rot_goal"

        if R_G_dis < 0.30 and wherestop == "starting point":
            wherestop = "stop_rot_home"

        if R_G_dis < 0.25:
            wherestop = "stop_adv"

        # 목표와 로봇사이 거리 스코어
        Rot = np.array(
            [[math.cos(current_angle.position.z * math.pi / 180), -math.sin(current_angle.position.z * math.pi / 180)],
             [math.sin(current_angle.position.z * math.pi / 180), math.cos(current_angle.position.z * math.pi / 180)]])
        path_len = np.round_((np.dot(xy_move_distance, Rot)), 4)  # 글로벌에서 본 경로거리를 구해서 4째자리까지 반올림
        r_g_path_len_x = goal_location_x - current_xyz.position.x + np.delete(path_len, 1, axis=2)
        r_g_path_len_y = goal_location_y - current_xyz.position.y + np.delete(path_len, 0, axis=2)
        r_g_dis = np.reshape(np.hypot(r_g_path_len_x, r_g_path_len_y), (10, mps_c, rps_c))
        r_g_score = r_g_dis[5]      # (1, rps_c), sqrt(x**2 + y**2)

    def lds_callback(self, scan):
        global goal_location_x
        global goal_location_y
        global goal_radian
        global DWA_mode
        global retry
        global r_g_score
        global stop_point
        global wherestop
        global SCAN_ran
        global n
        turn = False
        SCAN_ran = np.array(scan.ranges)
        scoremap = r_g_score - near_dis_score
        score_row_col = np.unravel_index(np.argmax(scoremap, axis=None), scoremap.shape)  # 최종 스코어를 골라서 인덱스를 구함

        turtle_vel.linear.x = Mps[score_row_col[0]]
        turtle_vel.angular.z = Radps[score_row_col[1]]
        # 만약 모든 범위가 10cm 보다 낮다면 turn
        if np.max(near_dis_score) == -100:
            turn = True
        if turn:
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = -1.0

        # 목표에 정면으로 바라보게
        if wherestop == "stop_rot_goal" or wherestop == "stop_rot_home":
            turtle_vel.linear.x = 0
            if -6 > (goal_radian - current_angle.position.z):
                turtle_vel.angular.z = -0.2
            if 6 < (goal_radian - current_angle.position.z):
                turtle_vel.angular.z = 0.2
            if -6 < (goal_radian - current_angle.position.z) < 6:
                turtle_vel.angular.z = 0
                wherestop = "stop_rot"

        # 목표와 20cm이내로 들게
        if wherestop == "stop_rot":
            turtle_vel.linear.x = 0.05
            turtle_vel.angular.z = 0
            # 20cm 이내로 들게 되면 curren_xyz 함수에서 wherestop = "stop_adv"

        ##########################mani가 집는 동작을 완료 후 control에서 뭘 어떻게 보냈는지 까먹음
        if wherestop == "stop_adv":
            stop_point.data = "stop"
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = 0
            if n == 0:
                goal_location_x = start_location_x
                goal_location_y = start_location_y
                n += 1

        if DWA_mode == "patrol" or DWA_mode == "home":
            self.publisher.publish(turtle_vel)
        else:
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = 0
            wherestop = "None"
        self.stop_point.publish(stop_point)


def main():
    rospy.init_node('DWA')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))

    rospy.spin()


if __name__ == "__main__":
    main()



