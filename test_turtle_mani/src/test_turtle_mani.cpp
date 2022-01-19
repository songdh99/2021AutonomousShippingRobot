#include "test_turtle_mani.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <iostream>

using namespace std; 

std::vector<double> kinematic_pose_sub;
ros::Publisher current_pose_pub;
ros::Publisher fin_act_pub;
ros::Publisher gripper_error_pub;
bool arrive_home;
int check_mode;
int check_gripper_error_value;

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
		target_pose.position.x = kinematics_pose.at(0) + 0.01; ////////////////////////////////////////////////////////////////////
		target_pose.position.y = kinematics_pose.at(1);
		target_pose.position.z = kinematics_pose.at(2) + 0.01;
		
		move_group_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
		ROS_INFO("x : %f\ny : %f\nz : %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(!my_plan.trajectory_.joint_trajectory.points.empty()){
			if (success == false)
				return false;

			move_group_->move();

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

    if(check_mode == 6){

		ROS_INFO("girpper value : %f", jointValues2.at(0));

		if((jointValues2.at(0)  >= -0.006) && (jointValues2.at(0) <= 0.005)) {
			error.data = false;
		}//제대로 잡은 경우
		else {
			error.data = true;
		} //제대로 잡지 못한 경우
		
		ROS_INFO("error : %d", error.data);
		gripper_error_pub.publish(error);
			
	}

	geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  
	
	current_pose_pub.publish(current_pose);
}

void OpenMani::demoSequence()
{
	std::vector<double> joint_angle;
	std::vector<double> gripper_value;
	bool b;

    std_msgs::Bool finish_act;
	finish_act.data = true;

    ros::Duration(3.0).sleep(); 


    if(check_mode == 0){
        ROS_INFO("wait 0");
        fin_act_pub.publish(finish_act);
    }


    else if(check_mode == 1){

        ROS_INFO("gripper open");

        gripper_value.push_back(0.01);
        b = setToolControl(gripper_value);

        if(b == true){
			fin_act_pub.publish(finish_act);
        }
        
        //ros::Duration(1.5).sleep();

    }
    else if(check_mode == 2){

        ROS_INFO("gripper close");

        gripper_value.push_back(-0.01);
        b = setToolControl(gripper_value);

        if(b == true){
			fin_act_pub.publish(finish_act);
        }

        //ros::Duration(1.5).sleep();

    }
    else if(check_mode == 3){
        ROS_INFO("move_to_aruco");

        b = setTaskSpacePath(kinematic_pose_sub, 2.0);
        
        if(b == true){
			fin_act_pub.publish(finish_act);
        }

        //ros::Duration(4.0).sleep();
    }
    else if(check_mode == 4){
        ROS_INFO("joint_move_to_place");
        joint_angle.push_back(-0.003);
        joint_angle.push_back(0.540);
        joint_angle.push_back(-0.179);
        joint_angle.push_back(0.626);
        b = setJointSpacePath(joint_angle, 2.0);

        if(b == true){
			fin_act_pub.publish(finish_act);
        }

        //ros::Duration(3.0).sleep();

    }

    else if(check_mode == 5){
        ROS_INFO("home_pose");

        joint_angle.push_back(-0.002);
        joint_angle.push_back(-1.578);
        joint_angle.push_back(1.029);
        joint_angle.push_back(0.610);

        b = setJointSpacePath(joint_angle, 2.0);

        if(b == true){
			fin_act_pub.publish(finish_act);
        }

        //ros::Duration(3.5).sleep();

    }


}

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	updateRobotState();
	ROS_INFO("check_mode number : %d", check_mode);

	if (!kinematic_pose_sub.empty())
		demoSequence();


}

void PoseCallback(const geometry_msgs::Pose &msg){

    kinematic_pose_sub.clear();

	kinematic_pose_sub.push_back(msg.position.x);
	kinematic_pose_sub.push_back(msg.position.y); 
	kinematic_pose_sub.push_back(msg.position.z);
}


void checkmode(const std_msgs::Int32& mgs)
{
	//ros::Duration(0.4).sleep();
	check_mode = mgs.data;
}
void check_gripper_error(const std_msgs::Int32 &msg){
    
    check_gripper_error_value = msg.data;
}


int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani OpenMani;
	if ( ! ros::master::check() )
		return false;
	
	ros::NodeHandle nh("");

    ros::Subscriber gripper_error_check_sub = nh.subscribe("gripper_error_check", 1, check_gripper_error);
	ros::Subscriber sub_ = nh.subscribe("a_about_m_pos", 1, PoseCallback); //sub aruco xyz
	ros::Timer publish_timer = nh.createTimer(ros::Duration(1.0), &OpenMani::publishCallback, &OpenMani);
	ros::Subscriber sub = nh.subscribe("check_mode", 1, checkmode); 

    current_pose_pub = nh.advertise<geometry_msgs::Pose>("mani_pos", 1); //arm xyz pub
    gripper_error_pub = nh.advertise<std_msgs::Bool>("mani_error", 1); //gripper error
    fin_act_pub = nh.advertise<std_msgs::Bool>("fin_act", 1); //finish act pub


	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}