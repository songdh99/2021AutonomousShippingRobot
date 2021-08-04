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

class OpenMani
{
private:
	ros::NodeHandle n;
	std::vector<std::string> joint_name;
	int cur_time;
	std_msgs::String current_mani_state; //?
	std::string planning_group_name;
	std::string planning_group_name2;
	moveit::planning_interface::MoveGroupInterface* move_group_;
	moveit::planning_interface::MoveGroupInterface* move_group2_;
	
	ros::Subscriber kinematic_pose_sub_;
	
public:
	OpenMani();
	~OpenMani();

	bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
	bool setToolControl(std::vector<double> joint_angle);
	void updateRobotState();
	bool setJointSpacePath(std::vector<double> joint_angle, double path_time);
	void publishCallback(const ros::TimerEvent&);
	void demoSequence();
};
