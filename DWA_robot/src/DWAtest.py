#!/home/pi/.pyenv/versions/rospy3/bin/python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


Mps = [0.20, 0.16, 0.12, 0.08, 0.04]
Radps = [0, 0.5, -0.5, 1.0, -1.0]   # 첫 원소는 무조건 0을 넣어야 함 (계산식이 다르기 때문)

SCANran = np.full((1, 360), 0)  # 360도 측정 거리값 초기화
five_Radps_scandistance = np.full((10, 1, 5), 0)    # 10스텝까지의 다섯개의 각속도에 따른 각도마다 스캔값 저장
# 속도, 각속도에 따라 도달하는 직선거리값을 step 마다 계산
# 한번 계산하고 계속 사용하기 위해 함수 밖에 작성
MpsAr = np.array(Mps).reshape(5, 1)
RadpsAr = np.delete(np.array(Radps), 0) # 각속도가 0일땐 거리계산식이 달라지므로 제외 후 따로 계산
step = 0.4 * np.arange(1, 11).reshape(10, 1, 1)
zeroRadpsAr = MpsAr * step   # 각속도가 0일때 (10, 5, 1)
distancestep = (2 * np.sin(RadpsAr * step / 2) / RadpsAr * MpsAr)   # (10, 5, 4)
fulldistancesteps = np.concatenate((zeroRadpsAr, distancestep), axis=2)  # (10, 5, 5)

angle160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
dg_angle160_Radps_step = np.int16(np.rint(angle160 + np.degrees(step * np.array(Radps))))     # (160, 10, 1, 5) 반올림 후 정수형환으로

# (Local)로봇 기준 이동시 x, y 이동거리 (10, 5, 5)
x_move_distance = np.concatenate((zeroRadpsAr, (distancestep * np.cos(90-(180-step*RadpsAr)/2))), axis=2)
y_move_distance = np.concatenate((np.zeros((10, 5, 1)), (distancestep * np.sin(90-(180-step*RadpsAr)/2))), axis=2)





class SelfDrive:

    def __init__(self, publisher):
        self.publisher = publisher

    def lds_callback(self, scan):#######만약 시간이 지나서 직선으로는 벽에 부딪힌걸로 되지만 벽을 넘는 가닥이라면..?
        turtle_vel = Twist()

        dfors = np.degrees(step * RadpsAr)   # degree or scan(10, 1, 5)
        dfors = np.int16(np.rint(dfors))   # 반올림 후 int형으로 변경
        # <SCANran> 측정 거리값 360
        global SCANran
        SCANran = np.array(scan.ranges)     # 튜플 타입인 scan.ranges를 행렬로 변환 대입

        # <five_Radps_distance>
        for i in range(0, 10):
            for k in range(0, 5):
                five_Radps_distance[i][1][k] = SCANran[dfors[i][1][k]]
        t_f_f = five_Radps_scandistance * np.ones(5, 1)
        true_false = t_f_f > fulldistancesteps  # (10, 5, 5) 계산값이 측정거리보다 낮아 부딪히지 않는다면 True

        # <passsec> 해당 가닥이 몇초동안 장애물에 부딪히지 않는지 계산
        passsec = np.int16(np.zeros((5, 5)))
        for i in range(0, 10):
            passsec = np.where(true_false[i], i, passsec)   # 부딪히기 바로 전 step을 저장

        # <maxpass_neardis> 이동했을 시점에서 가장 가까운 장애물과의 거리 계산
        # (160, 10, 1, 5) 각도를 스캔한 거리값으로 변경
        a_R_s_scandistance = np.where(True, SCANran[dg_angle160_Radps_step], SCANran[dg_angle160_Radps_step])
        # (160, 10, 5, 5)
        neardis160 = np.sqrt((a_R_s_scandistance * np.sin(np.radians(dg_angle160_Radps_step)))**2 + (fulldistancesteps - a_R_s_scandistance * abs(np.cos(np.radians(dg_angle160_Radps_step))))**2)
        # (10, 5, 5)
        neardis = np.min(neardis160, axis=0)
        maxpass_neardis = np.zeros((5, 5))
        for i in range(0, 10):
            for j in range(0, 5):
                k = passsec[i][j]
                maxpass_neardis[i][j] = neardis[k][i][j]    # (5, 5)
        mp_nd = np.where(maxpass_neardis > 0.2, 0.2, maxpass_neardis)     # 20cm가 넘는것은 20cm로 만듦
        mp_nd_score = np.argsort(mp_nd)
        """
        # goal과 robot사이의 거리
        global goal_location_x, goal_location_y
        robot_to_goal_x = x_move_distance - goal_location_x
        robot_to_goal_y = y_move_distance - goal_location_y
        r_t_g_dis = np.hypot(robot_to_goal_x, robot_to_goal_y)  # sqrt(x**2 + y**2)
        r_t_g_dis_penalty = np.argsort(r_t_g_dis)  # (10, 5, 5) robot과 goal 사이의 거리를 작은 순서대로 순위매김
        """
        scoremap = mp_nd_score # r_t_g_dis_penalty를 빼거나 해야됨
        score_row_col = np.unravel_index(np.argmax(scoremap, axis=None), scoremap)
        turtle_vel.linear.x = Mps[score_row_col[0]]
        turtle_vel.angular.z = Radps[score_row_col[1]]

        self.publisher.publish(turtle_vel)










def main():

    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()


if __name__ == "__main__":
    main()


