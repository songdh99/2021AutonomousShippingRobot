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

SCANran = np.full((1, 360), 0)  # 360도 측정 거리값 초기화
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
o = 0
n = 0
DWA_mode = String()


class SelfDrive:

    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_point = rospy.Publisher('stop_point', String, queue_size=1)
        rospy.Subscriber('DWA_pub', String, self.check_mode)
        rospy.Subscriber('current_xyz', Pose, self.current_xyz)
        rospy.Subscriber('current_angle', Pose, self.current_angle)

    def current_angle(self, angle):
        current_angle.position.z = angle.position.z * 180 / math.pi  # angle.position.z가 360도가 1로 나타남
        if current_angle.position.z < 0:
            current_angle.position.z += 360

    def current_xyz(self, xyz):
        global retry
        global r_g_score
        global stop_point
        global goal_location_x
        global goal_location_y
        global start_location_x
        global start_location_y
        global goal_radian
        global wherestop
        current_xyz.position.x = xyz.position.x
        current_xyz.position.y = xyz.position.y
        retry += 1
        if retry == 1:
            wherestop = "goal point"
            start_location_x = current_xyz.position.x
            start_location_y = current_xyz.position.y

        x = goal_location_x - current_xyz.position.x
        y = goal_location_y - current_xyz.position.y
        goal_radian = math.atan2(y, x) * 180 / math.pi
        if goal_radian < 0:
            goal_radian += 360
        RtoGdis = np.hypot(goal_location_x - current_xyz.position.x, goal_location_y - current_xyz.position.y)
        print('RtoGdis', RtoGdis)
        if RtoGdis < 0.30 and wherestop == "goal point":
            wherestop = "stop_rot_goal"

            # goal_radian = math.atan2(y, x)

        if RtoGdis < 0.30 and wherestop == "starting point":
            wherestop = "stop_rot_home"
            # goal_radian = math.atan2(y, x)

        if RtoGdis < 0.25:
            wherestop = "stop_adv"

        # 목표와 로봇사이 거리 스코어
        Rot = np.array(
            [[math.cos(current_angle.position.z * math.pi / 180), -math.sin(current_angle.position.z * math.pi / 180)],
             [math.sin(current_angle.position.z * math.pi / 180), math.cos(current_angle.position.z * math.pi / 180)]])
        path_len = np.round_((np.dot(xy_move_distance, Rot)), 4)  # 글로벌에서 본 경로거리를 구해서 4째자리까지 반올림
        r_g_path_len_x = goal_location_x - current_xyz.position.x + np.delete(path_len, 1, axis=2)
        r_g_path_len_y = goal_location_y - current_xyz.position.y + np.delete(path_len, 0, axis=2)
        r_g_dis = np.reshape(np.hypot(r_g_path_len_x, r_g_path_len_y), (10, mps_c, rps_c))
        r_g_score = RtoGdis - r_g_dis[5]  # np.amin(r_g_dis, axis=0)#.reshape(1, -1)    # (1, rps_c), sqrt(x**2 + y**2)
        # print('', r_g_dis[3])

    def lds_callback(self, scan):  #######만약 시간이 지나서 직선으로는 벽에 부딪힌걸로 되지만 벽을 넘는 가닥이라면..?
        global stop_point
        global SCANran
        global goal_location_x
        global goal_location_y
        global start_location_x
        global start_location_y
        global goal_radian
        global current_angle
        global wherestop
        global DWA_mode
        global n

        turtle_vel = Twist()
        turn = False

        dfors = np.degrees(step * np.array(Radps))  # degree for scan(10, 1, rps_c)
        dfors = np.int32(np.rint(dfors))  # 반올림 후 int형으로 변경
        # <SCANran> 측정 거리값 360
        SCANran = np.array(scan.ranges)  # 튜플 타입인 scan.ranges를 행렬로 변환 대입

        # <five_Radps_scandistance>
        for i in range(0, 10):
            for k in range(0, rps_c):
                five_Radps_scandistance[i][0][k] = SCANran[dfors[i][0][k]]
        t_f_f = five_Radps_scandistance * np.ones((mps_c, 1))
        true_false = t_f_f > fulldistancesteps  # (10, mps_c, rps_c) 계산값이 측정거리보다 낮아 부딪히지 않는다면 True

        # <passsec> (mps_c, rps_c) 해당 가닥이 몇초동안 장애물에 부딪히지 않는지 계산
        passsec = np.int32(np.zeros((mps_c, rps_c)))
        for i in range(0, 10):
            passsec = np.where(true_false[i], i, passsec)  # 부딪히기 바로 전 step을 저장

        # <pass_distance> (mps_c, rps_c) 부딪히지 않고 이동하는 거리
        pass_distance = passsec * np.array(Mps).reshape(mps_c, 1)

        # <maxpass_neardis> 이동했을 시점에서 가장 가까운 장애물과의 거리 계산
        # (160, 10, 1, rps_c) 각도를 스캔한 거리값으로 변경
        a_R_s_scandistance = np.where(True, SCANran[dg_angle160_Radps_step], SCANran[dg_angle160_Radps_step])
        # (160, 10, mps_c, rps_c)
        neardis160 = np.sqrt((a_R_s_scandistance * np.sin(np.radians(dg_angle160_Radps_step))) ** 2 + (
                    fulldistancesteps - a_R_s_scandistance * abs(np.cos(np.radians(dg_angle160_Radps_step)))) ** 2)
        # (10, mps_c, rps_c)
        neardis = np.amin(neardis160, axis=0)
        maxpass_neardis = np.zeros((mps_c, rps_c))
        for i in range(0, mps_c):
            for j in range(0, rps_c):
                k = (passsec[i][j] - 2) % 1
                maxpass_neardis[i][j] = neardis[k][i][j]  # (mps_c, rps_c)
        mp_nd = np.where(maxpass_neardis > 0.30, 0.5, maxpass_neardis)  # 30cm가 넘는 것은 30cm로 만듦
        mp_nd_score = np.where(mp_nd < 0.15, -100, mp_nd)  # 10cm 보다 낮은 것은 -1로 만듦
        # 만약 모든 범위가 10cm 보다 낮다면 turn
        if np.max(mp_nd_score) == -100:
            turn = True

        # 최종 스코어 (장애물을 피하는 스코어맵의 top5를 구해 그 중 목표물과 가장 가까워 지는 스코어 선택)

        scoremap = 10 * mp_nd_score + pass_distance + 0.00003 * r_g_score
        score_row_col = np.unravel_index(np.argmax(scoremap, axis=None), scoremap.shape)  # 최종 스코어를 골라서 인덱스를 구함

        ####
        #print('rr\n', scoremap)
        #print('scoremap', scoremap)
        print('goal{}, current{}, @@@{}'.format(goal_radian, current_angle.position.z, (goal_radian - current_angle.position.z)))
        #print('goal_radian', goal_radian)

        turtle_vel.linear.x = Mps[score_row_col[0]]
        turtle_vel.angular.z = Radps[score_row_col[1]]

        if turn:
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = -1.0
            ########## stop_point로 보내는 값 바꿔야됨

        # 목표에 정면으로 바라보게
        if wherestop == "stop_rot_goal" or wherestop == "stop_rot_home":
            turtle_vel.linear.x = 0
            if -2.5 > (goal_radian - current_angle.position.z):
                turtle_vel.angular.z = -0.14
            if 2.5 < (goal_radian - current_angle.position.z):
                turtle_vel.angular.z = 0.14
            if -2.5 < (goal_radian - current_angle.position.z) < 2.5:
                turtle_vel.angular.z = 0
                wherestop = "stop_rot"

        # 목표와 20cm이내로 들게
        if wherestop == "stop_rot":
            turtle_vel.linear.x = 0.1
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

        #if DWA_mode == "patrol" or DWA_mode == "home":
            #self.publisher.publish(turtle_vel)
        self.stop_point.publish(stop_point)

        #print('wherestop {}, stop_point {}, DWA_mode {}'.format(wherestop, stop_point.data, DWA_mode))

    def check_mode(self, DWA_pub):
        global o
        global DWA_mode
        global wherestop
        DWA_mode = DWA_pub.data  ###########고쳐야됨
        if DWA_mode == "home" and o == 0:
            wherestop = "starting point"
            o = 1


def main():
    rospy.init_node('DWA')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))

    # rate = rospy.Rate(1000)
    # rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()



