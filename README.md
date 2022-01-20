# 2021AutonomousShippingRobot

2020.12.01 ~ 2021.11.09

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/52944554/150262189-f1cf844f-4d03-4732-81d3-c755e4a0a5b3.gif)


## 기본적으로 필요한 패키지

```
sudo apt install ros-melodic-joy\
  ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard\
  ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino \
  ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs \ 
  ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf \
  ros-melodic-xacro ros-melodic-compressed-image-transport \
  ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers \
  ros-melodic-gazebo* ros-melodic-moveit* ros-melodic-industrial-core \
  ros-melodic-ros-control* ros-melodic-control*  ros-melodic-moveit*
```



```
git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
```



## 패키지 설명

- DWA



- aruco



- control



- navigation



- slam_gmapping



- test_turtle_mani : manipulator를 제어하는 패키지



- tf



## 명령어

1. roscore 실행	

```
roscore
```

2. (새 창) 라즈베리파이 원격 접속

```
ssh name@ip
```

3.  bringup

```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

4. (새 창) manipulator bringup

```
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

5. (새 창) movegroup

``` 
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

6. (새 창)

```
roslaunch navigation gmapping.launch 
```

7. (새 창)

```
rosrun navigation send_current_xyz.py
```

8. (새 창) test_turtle_mani 실행

```
rosrun test_turtle_mani test_turtle_mani
```

9. (새 창) test_turtle_mani_with_kine 실행

```
rosrun test_turtle_mani test_turtle_mani_with_kine
```

10. (새 창)

```
rosrun tf tf_pub.py 
```

11. (새 창)

```
rosrun tf tf_sub.py
```

12. (새 창)

```
rosrun control control_test.py
```

13. (새 창)

```
rosrun DWA_robot DWAtest.py
```







## 문제점

- 노드를 일일이 켜야 한다.
- manipulator가 자주 꺼지는데 그 이유를 찾지 못했다.
- 가상환경을 만들지 않았다.
