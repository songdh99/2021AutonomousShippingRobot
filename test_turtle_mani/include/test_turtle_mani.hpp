#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>

#include <time.h>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

#define PICK_UP_GRIPPER_OPEN 0
#define PICK_UP_MANI_MOVE 1
#define PICK_UP_GRIPPER_CLOSE 2
#define PICK_UP_MOVE_HOME 3
#define MOVE_BASE 4
#define PICK_DOWN_GRIPPER_OPEN 5
#define PICK_DOWN_MOVE_HOME 6

class OpenMani
{
private:
	ros::NodeHandle n;
	std::vector<std::string> joint_name;
	int small_box_count;
	int pick_large_box_count;
	int wait_bot_count;
	int release_box_count;
	int count;
	int check;
	int cur_time;
	int check_mode;
	int error_mode;
	int bot_ready;
	int start_pose;
	int ar_marker_id;
	std_msgs::String current_mani_state;
	std::string planning_group_name;
	std::string planning_group_name2;
	moveit::planning_interface::MoveGroupInterface* move_group_;
	moveit::planning_interface::MoveGroupInterface* move_group2_;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	ros::Subscriber kinematic_pose_sub_;
	ros::Subscriber ar_marker_sub_;
	ros::Subscriber lift_bot_state_sub_;
	ros::Publisher current_mani_state_pub_;
	
public:
	OpenMani();
	~OpenMani();

	bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
	bool setToolControl(std::vector<double> joint_angle);
	void updateRobotState();
	bool setJointSpacePath(std::vector<double> joint_angle, double path_time);
	void init_sub_pub();

	void publishCallback(const ros::TimerEvent&);
	void Wait_Bot();
	void Release_Box();
	void Pick_Up_Small_Box();
	void Pick_Up_Large_Box();
	void demoSequence();
};
	
