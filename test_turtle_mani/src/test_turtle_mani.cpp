#include "test_turtle_mani.hpp"

std::vector<double> kinematic_pose_sub;
ros::Publisher pub_;
bool arrive_home;

OpenMani::OpenMani()
:n(""),
 count(0),
 check_mode(0),
 start_pose(0),
 check(0),
 error_mode(0)
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
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	if(!kinematics_pose.empty()){
		geometry_msgs::Pose target_pose;
		target_pose.position.x = kinematics_pose.at(0);
		target_pose.position.y = kinematics_pose.at(1);
		target_pose.position.z = kinematics_pose.at(2);
		
		move_group_->setPositionTarget(
			target_pose.position.x,
			target_pose.position.y,
			target_pose.position.z);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(!my_plan.trajectory_.joint_trajectory.points.empty()){
			if (success == false)
				return false;

			move_group_->move();

			cur_time = time(0);

			spinner.stop();
			return true;
		}
	}
}

bool OpenMani::setJointSpacePath(std::vector<double> joint_angle, double path_time)
{
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	// Next get the current set of joint values for the group.
	const robot_state::JointModelGroup* joint_model_group =
	move_group_->getCurrentState()->getJointModelGroup("arm");

	moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
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

	cur_time = time(0);

	spinner.stop();
	return true;
}

bool OpenMani::setToolControl(std::vector<double> joint_angle)
{
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	// Next get the current set of joint values for the group.
	const robot_state::JointModelGroup* joint_model_group =
	move_group2_->getCurrentState()->getJointModelGroup("gripper");
    
	moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
	joint_group_positions[0] = joint_angle.at(0);  // radians
	move_group2_->setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	if (success == false)
		return false;

	move_group2_->move();

	cur_time = time(0);

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

	geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose; 
	ros::Publisher pos_pub = n.advertise<geometry_msgs::Pose>("mani_pos", 10); 
	
	pos_pub.publish(current_pose);
}

void OpenMani::demoSequence()
{
	std::vector<double> joint_angle;
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;
	int add_time = 0;
	int fin = 0;
	std_msgs::Bool fin_pick_up;
	std_msgs::Bool fin_mani;
	std_msgs::Bool mani_plan;
	std_msgs::Bool mani_error_plan;
	bool b;

	add_time = time(0);
	if((add_time-cur_time) >= 1){
		if (!kinematic_pose_sub.empty()) count ++;
		else mani_plan.data = true;
	}	


	if(check_mode == 0){
		joint_angle.push_back( 0.000 );
		joint_angle.push_back( -1.553 );
		joint_angle.push_back( 1.204 );
		joint_angle.push_back( 0.610 );
		ROS_INFO("up");
		b = setJointSpacePath(joint_angle, 2.0);
		add_time = time(0);  
		check_mode ++;           
		
	}
	
}

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	updateRobotState();
    ROS_INFO("publishCallback");

	if (!kinematic_pose_sub.empty())
		demoSequence();
}

void ArriveHomeCallback(const std_msgs::Bool &msg){

	arrive_home = msg.data;
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani OpenMani;

	if ( ! ros::master::check() )
		return false;
	
	ros::NodeHandle n;
	
	ROS_INFO("main");



	kinematic_pose_sub.push_back(0.0);
	kinematic_pose_sub.push_back(0.0); 
	kinematic_pose_sub.push_back(0.0);

	ros::Timer publish_timer = n.createTimer(ros::Duration(1.3), &OpenMani::publishCallback, &OpenMani);
	
	pub_ = n.advertise<geometry_msgs::Pose>("mani_pose", 10);
	ROS_INFO("11");

	while (ros::ok())
	{
		ros::spinOnce();
		
	}
	
}
