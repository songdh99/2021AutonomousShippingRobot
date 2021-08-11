#include "test_turtle_mani.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <iostream>

using namespace std; 

std::vector<double> kinematic_pose_sub;
ros::Publisher pub_;
ros::Publisher pubb;
ros::Publisher error_pub;
ros::Publisher g_error_pub;
bool arrive_home;
int check_mode;

std_msgs::Bool error;

OpenMani::OpenMani()
:n("")
{
	joint_name.push_back("joint1");
	joint_name.push_back("joint2"); 
	joint_name.push_back("joint3"); 
	joint_name.push_back("joint4"); 
	
	  // Move group arm
	planning_group_name = "arm";
	
	// Move group gripper
	planning_group_name2 = "gripper";
	
	move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
	move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);
}


OpenMani::~OpenMani()
{
	if (ros::isStarted()) 
	{
		ros::shutdown();
		ros::waitForShutdown();
	}
}

bool OpenMani::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
	ROS_INFO("setTaskSpacePath");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	if(!kinematics_pose.empty()){
		geometry_msgs::Pose target_pose;
		target_pose.position.x = kinematics_pose.at(0);
		target_pose.position.y = kinematics_pose.at(1);
		target_pose.position.z = kinematics_pose.at(2);
		
		move_group_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(!my_plan.trajectory_.joint_trajectory.points.empty()){
			if (success == false)
				return false;

			move_group_->move();

			//cur_time = time(0);

			spinner.stop();
			return true;
		}
	}
}

bool OpenMani::setJointSpacePath(std::vector<double> joint_angle, double path_time)
{
	ROS_INFO("setJointSpacePath");
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	const robot_state::JointModelGroup* joint_model_group =
	move_group_->getCurrentState()->getJointModelGroup("arm");

	moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	joint_group_positions[0] = joint_angle.at(0);  // radians
	joint_group_positions[1] = joint_angle.at(1);  // radians
	joint_group_positions[2] = joint_angle.at(2);  // radians
	joint_group_positions[3] = joint_angle.at(3);  // radians
	move_group_->setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	
	if (success == false)
		return false;

	move_group_->move();

	//cur_time = time(0);

	spinner.stop();
	return true;
}

bool OpenMani::setToolControl(std::vector<double> joint_angle)
{
	ROS_INFO("setToolControl");
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	const robot_state::JointModelGroup* joint_model_group =
	move_group2_->getCurrentState()->getJointModelGroup("gripper");
    
	moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	
	joint_group_positions[0] = joint_angle.at(0);  // radians
	move_group2_->setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	if (success == false)
		return false;

	move_group2_->move();

	//cur_time = time(0);

	spinner.stop();
	return true;

}

void OpenMani::updateRobotState()
{
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	std::vector<double> jointValues = move_group_->getCurrentJointValues();
	std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
	std::vector<double> temp_angle;
	std::vector<double> temp_position;
	std_msgs::Bool gripper_error;

	temp_angle.push_back(jointValues2.at(0));

	ROS_INFO("girpper : %f", jointValues2.at(0));


	if(check_mode == 6){

		if((jointValues2.at(0)  >= -0.006) && (jointValues2.at(0) <= 0.005)) {
			error.data = false;
			ROS_INFO("gggg fffffffffff 000000000000 : %d", error.data);
		}//제대로 잡은 경우
		else {
			error.data = true;
			ROS_INFO("gggg ssssssssssssssssssssss 111111111111 : %d", error.data);
		} //제대로 잡지 못한 경우
		
		ROS_INFO("error : %d", error.data);
		error_pub.publish(error);
			
	}
	geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  

	
	pub_.publish(current_pose);
    
}

void OpenMani::demoSequence()
{
	std::vector<double> joint_angle;
	std::vector<double> gripper_value;
	bool b;

	std_msgs::Bool T;
	T.data = true;

	error.data = false;

	ros::Duration(3.0).sleep();

	if(check_mode == 0){
		ROS_INFO("wait 0");
		pubb.publish(T);
	}

	if(check_mode == 1){
        
        gripper_value.push_back(0.01);
        setToolControl(gripper_value);
        
        ROS_INFO("gripper open");
        ros::Duration(1.5).sleep();

        pubb.publish(T);
	}

	if(check_mode == 2)
	{
        gripper_value.push_back(-0.01);
        setToolControl(gripper_value);
        ROS_INFO("gripper close");
        ros::Duration(1.5).sleep();

        pubb.publish(T);
		ROS_INFO("%d", T);
	}

    if(check_mode == 3)
	{
        ROS_INFO("move");
		// kinematic_pose_sub.clear();
        // kinematic_pose_sub.push_back(0.164);
        // kinematic_pose_sub.push_back(0.000); 
        // kinematic_pose_sub.push_back(0.209);

        b = setTaskSpacePath(kinematic_pose_sub, 2.0); //add b.

		if(b == true){

			ROS_INFO("move_to_aruco");
			ros::Duration(3.5).sleep();
			//error.data = false;
			//error_pub.publish(error);
			pubb.publish(T);

		}
		else if (b==false){
			ROS_INFO("mani_error_pub");
			//error.data = true;
			//error_pub.publish(error);
			
			pubb.publish(T);	
		}
	}

	if(check_mode == 4)
	{
        joint_angle.push_back(1.468);
        joint_angle.push_back(-0.506);
        joint_angle.push_back(0.578);
        joint_angle.push_back(0.733);
        setJointSpacePath(joint_angle, 2.0);
        
        ROS_INFO("joint_move");
        ros::Duration(3.0).sleep();

        pubb.publish(T);
	}	

	if(check_mode == 5){
        ROS_INFO("zero");

        joint_angle.push_back(-0.002);
        joint_angle.push_back(-1.578);
        joint_angle.push_back(1.029);
        joint_angle.push_back(0.610);

		// joint_angle.push_back(0.094);
        // joint_angle.push_back(-0.965);
        // joint_angle.push_back(0.328);
        // joint_angle.push_back(0.707);

        setJointSpacePath(joint_angle, 2.0);
        ros::Duration(3.5).sleep();
        
        pubb.publish(T);
    } 
}

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	updateRobotState();
	ROS_INFO("check_mode in publish : %d", check_mode);

	if (!kinematic_pose_sub.empty())
		demoSequence();
}

void PoseCallback(const geometry_msgs::Pose &msg){

	kinematic_pose_sub.clear();

	kinematic_pose_sub.push_back(msg.position.x);
	kinematic_pose_sub.push_back(msg.position.y); 
	kinematic_pose_sub.push_back(msg.position.z);
	//ROS_INFO("sub x : %f", msg.position.x);
}

void checkmode(const std_msgs::Int32& mgs)
{
	ros::Duration(0.4).sleep();
	check_mode = mgs.data;
	ROS_INFO("check_mode : %d", check_mode);
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani OpenMani;
	if ( ! ros::master::check() )
		return false;
	
	ros::NodeHandle nh("");

	kinematic_pose_sub.clear();
	kinematic_pose_sub.push_back(0.0);
	kinematic_pose_sub.push_back(0.0); 
	kinematic_pose_sub.push_back(0.0);

	ros::Subscriber sub_ = nh.subscribe("a_about_m_pos", 10, PoseCallback); //sub aruco xyz
	ros::Timer publish_timer = nh.createTimer(ros::Duration(1.0), &OpenMani::publishCallback, &OpenMani);
	ros::Subscriber sub = nh.subscribe("check_mode", 10, checkmode); //it tells you what to do

	g_error_pub = nh.advertise<std_msgs::Bool>("gripper_error_pub", 10);
	pub_ = nh.advertise<geometry_msgs::Pose>("mani_pos", 10); //arm xyz pub
    pubb = nh.advertise<std_msgs::Bool>("fin_act", 10); //동작 끝나면 끝났다고 전송하는 pub
	error_pub = nh.advertise<std_msgs::Bool>("mani_error", 10); //when arm can not reach
	
	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}
