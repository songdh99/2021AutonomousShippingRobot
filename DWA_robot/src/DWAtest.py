#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# 속도, 각속도의 개수
mps_c = 5
rps_c = 13

Mps = [0.15, 0.13, 0.11, 0.09, 0.07]
Radps = [0, 0.3, -0.3, 0.5, -0.5, 0.6, -0.6, 0.7, -0.7, 0.8, -0.8, 0.9, -0.9]   # 첫 원소는 무조건 0을 넣어야 함 (계산식이 다르기 때문)

SCANran = np.full((1, 360), 0)  # 360도 측정 거리값 초기화
five_Radps_scandistance = np.full((10, 1, rps_c), 0.)    # 10스텝까지의 다섯개의 각속도에 따른 각도마다 스캔값 저장
# 속도, 각속도에 따라 도달하는 직선거리값을 step 마다 계산
# 한번 계산하고 계속 사용하기 위해 함수 밖에 작성
MpsAr = np.array(Mps).reshape(mps_c, 1)
RadpsAr = np.delete(np.array(Radps), 0)  # 각속도가 0일땐 거리계산식이 달라지므로 제외 후 따로 계산
step = 0.1 * np.arange(1, 11).reshape(10, 1, 1)
zeroRadpsAr = MpsAr * step   # 각속도가 0일때 (10, mps_c, 1)
distancestep = (2 * np.sin(RadpsAr * step / 2) / RadpsAr * MpsAr)   # (10, mps_c, rps_c-1)
fulldistancesteps = np.concatenate((zeroRadpsAr, distancestep), axis=2) + 0.25  # (10, mps_c, rps_c) 로봇의 크기보정을 위해 + 0.2

angle160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
dg_angle160_Radps_step = np.int32(np.rint(angle160 + np.degrees(step * np.array(Radps))))     # (160, 10, 1, rps_c) 반올림 후 정수형으로 변환

# (Local)로봇 기준 이동시 x, y 이동거리 (10, mps_c, rps_c)
x_move_distance = np.concatenate((zeroRadpsAr, (distancestep * np.cos(90-(180-step*RadpsAr)/2))), axis=2)
y_move_distance = np.concatenate((np.zeros((10, mps_c, 1)), (distancestep * np.sin(90-(180-step*RadpsAr)/2))), axis=2)





class SelfDrive:

    def __init__(self, publisher):
        self.publisher = publisher

    def lds_callback(self, scan):#######만약 시간이 지나서 직선으로는 벽에 부딪힌걸로 되지만 벽을 넘는 가닥이라면..?
        turtle_vel = Twist()

        dfors = np.degrees(step * np.array(Radps))   # degree for scan(10, 1, rps_c)
        dfors = np.int32(np.rint(dfors))   # 반올림 후 int형으로 변경
        # <SCANran> 측정 거리값 360
        global SCANran
        SCANran = np.array(scan.ranges)     # 튜플 타입인 scan.ranges를 행렬로 변환 대입

        # <five_Radps_scandistance>
        for i in range(0, 10):
            for k in range(0, rps_c):
                five_Radps_scandistance[i][0][k] = SCANran[dfors[i][0][k]]
        t_f_f = five_Radps_scandistance * np.ones((mps_c, 1))
        true_false = t_f_f > fulldistancesteps  # (10, mps_c, rps_c) 계산값이 측정거리보다 낮아 부딪히지 않는다면 True

        # <passsec> (mps_c, rps_c) 해당 가닥이 몇초동안 장애물에 부딪히지 않는지 계산
        passsec = np.int32(np.zeros((mps_c, rps_c)))
        for i in range(0, 10):
            passsec = np.where(true_false[i], i, passsec)   # 부딪히기 바로 전 step을 저장

        # <pass_distance> (mps_c, rps_c) 부딪히지 않고 이동하는 거리
        pass_distance = passsec * np.array(Mps).reshape(mps_c, 1)


        # <maxpass_neardis> 이동했을 시점에서 가장 가까운 장애물과의 거리 계산
        # (160, 10, 1, rps_c) 각도를 스캔한 거리값으로 변경
        a_R_s_scandistance = np.where(True, SCANran[dg_angle160_Radps_step], SCANran[dg_angle160_Radps_step])
        # (160, 10, mps_c, rps_c)
        neardis160 = np.sqrt((a_R_s_scandistance * np.sin(np.radians(dg_angle160_Radps_step)))**2 + (fulldistancesteps - a_R_s_scandistance * abs(np.cos(np.radians(dg_angle160_Radps_step))))**2)
        # (10, mps_c, rps_c)
        neardis = np.min(neardis160, axis=0)
        maxpass_neardis = np.zeros((mps_c, rps_c))
        for i in range(0, mps_c):
            for j in range(0, rps_c):
                k = (passsec[i][j] - 2) % 1
                maxpass_neardis[i][j] = neardis[k][i][j]    # (mps_c, rps_c)
        mp_nd = np.where(maxpass_neardis > 0.30, 0.30, maxpass_neardis)     # 20cm가 넘는것은 20cm로 만듦

        """
        # goal과 robot사이의 거리
        global goal_location_x, goal_location_y
        robot_to_goal_x = x_move_distance - goal_location_x
        robot_to_goal_y = y_move_distance - goal_location_y
        r_t_g_dis = np.hypot(robot_to_goal_x, robot_to_goal_y)  # sqrt(x**2 + y**2)
        """

        scoremap = 10 * mp_nd + pass_distance  # r_t_g_dis를 빼거나 해야됨
        score_row_col = np.unravel_index(np.argmax(scoremap, axis=None), scoremap.shape)

        ####
        print('pass_distance\n', passsec)
        print('mp_nd\n', mp_nd)

        turtle_vel.linear.x = Mps[score_row_col[0]]
        turtle_vel.angular.z = Radps[score_row_col[1]]

        self.publisher.publish(turtle_vel)


def main():
    rospy.init_node('DWA')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()


if __name__ == "__main__":
    main()



