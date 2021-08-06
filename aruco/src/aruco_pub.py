#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import cv2
import glob
import os
import rospy
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)


def cal():
    allCorners = []
    allIds = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    #glob() 안에 사진 위치를 올바르게 적어야함
    images = glob.glob('/home/x/workspace/aruco/aruco/src/camera_cal_img/cal*.png')
    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        imsize = gray.shape

    return allCorners, allIds, imsize


def calibrate_charuco(allCorners, allIds, imsize):
    cameraMatrixInit = np.array([[2714., 0., imsize[1] / 2],
                                 [0., 2714., imsize[0] / 2.],
                                 [0., 0., 1.]])
    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def detect_marker(mtx, dist):
    os.system('sudo modprobe bcm2835-v4l2')
    #cam = cv2.VideoCapture(0)
    cam = cv2.VideoCapture(-1)
    param = cv2.aruco.DetectorParameters_create()
    aruco_pub = rospy.Publisher('aruco_xyzw', Pose, queue_size=10)
    check_pub = rospy.Publisher('check_aruco', Bool, queue_size=10)
    rate = rospy.Rate(10)
    aruco = Pose()
    check = Bool()
    if cam.isOpened():
        while not rospy.is_shutdown():
            _, frame = cam.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)
            if np.all(ids != None):
                check.data = True
                #rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(coners, 0.04, mtx, dist)
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, 0.02, mtx, dist)
                frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[0], tvecs[0], 0.02)
                rvecs_msg = rvecs.tolist()
                tvecs_msg = tvecs.tolist()
                aruco.orientation.x = rvecs_msg[0][0][0]
                aruco.orientation.y = rvecs_msg[0][0][1]
                aruco.orientation.z = rvecs_msg[0][0][2]
                aruco.position.x = tvecs_msg[0][0][0]
                aruco.position.y = tvecs_msg[0][0][1]
                aruco.position.z = tvecs_msg[0][0][2]
                rospy.loginfo("{}".format(aruco))
                aruco_pub.publish(aruco)
            else:
                check.data = False
            check_pub.publish(check)
            rate.sleep()
            frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
            cv2.imshow('video', frame)
            k = cv2.waitKey(30)

        cam.release()


def main():
    rospy.init_node("aruco_pub")
    allCorners, allIds, imsize = cal()
    ret, mtx, dist, rvec, tvec = calibrate_charuco(allCorners, allIds, imsize)
    detect_marker(mtx, dist)


if __name__ == "__main__":
    main()

