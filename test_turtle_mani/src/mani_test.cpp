#include "test_turtle_mani.hpp"

std::vector<double> kinematic_pose_sub;
ros::Publisher mani_plan_status_pub_;
ros::Publisher mani_error_pub_;
ros::Publisher pick_up_pub_;
ros::Publisher pick_down_pub_;
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

	temp_angle.push_back(jointValues.at(0));
	temp_angle.push_back(jointValues.at(1));
	temp_angle.push_back(jointValues.at(2));
	temp_angle.push_back(jointValues.at(3));
	temp_angle.push_back(jointValues2.at(0));

	geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  

	temp_position.push_back(current_pose.position.x);
	temp_position.push_back(current_pose.position.y);
	temp_position.push_back(current_pose.position.z);
	
	pub_.publish(current_pose);
}

void OpenMani::demoSequence()
{
	std::vector<double> joint_angle;
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;
	int add_time = 0;
	std_msgs::Bool fin_pick_up;
	std_msgs::Bool fin_mani;
	std_msgs::Bool mani_plan;
	std_msgs::Bool mani_error_plan;
	bool b;

	switch(count){

	case 0: 
		if(check_mode == PICK_UP_GRIPPER_OPEN)
		{
			mani_plan.data = false;
			fin_mani.data = false;
			pick_down_pub_.publish(fin_mani);

			gripper_value.push_back(0.01);
			setToolControl(gripper_value);
			check_mode ++;
			ROS_INFO("case 0: gripper open");
		}

		add_time = time(0);
		if((add_time-cur_time) >= 1){
			if (!kinematic_pose_sub.empty())
				count ++;
				
			else{	
				mani_plan.data = true;
				mani_plan_status_pub_.publish(mani_plan);
			}
		}	
		break;

	case 1: 
		if(check_mode == PICK_UP_MANI_MOVE)
		{
			if (!kinematic_pose_sub.empty()){
				b = setTaskSpacePath(kinematic_pose_sub, 2.0);
				check_mode ++;
				ROS_INFO("case 1: move mani (pick up)");
                                kinematic_pose_sub.clear();
			}
			else {
				b = false;
                                //kinematic_pose_sub.clear();
                                ROS_INFO("failed");
                       }
		}

		if (b==true){
			add_time = time(0);
			if((add_time-cur_time) >= 6)
				count ++;
		}

		else if (b==false){
			ROS_INFO("%d", mani_plan);
			mani_plan.data = true;
			mani_plan_status_pub_.publish(mani_plan);
			
			count = 0;
			check_mode = 0;
			kinematic_pose_sub.clear();
			fin_pick_up.data = false;
			mani_plan.data = false;

			
			/*if(error_mode == 1){
				joint_angle.push_back( 0.000 );
				joint_angle.push_back( -1.56 );
				joint_angle.push_back( 1.100 );
				joint_angle.push_back( 0.500 );
				setJointSpacePath(joint_angle, 2.0);
				error_mode ++;
				ROS_INFO("case 1: move mani --error--");
			}
				
			add_time = time(0);
			if((add_time-cur_time) >= 6){
				count = 0;
				check_mode = 0;
				kinematic_pose_sub.clear();
				fin_pick_up.data = false;
				mani_plan.data = false;
				mani_error_plan.data = true;
				mani_error_pub_.publish(mani_error_plan);
			}*/
		}
		break;
	case 2:
		if(check_mode == PICK_UP_GRIPPER_CLOSE)
		{
			gripper_value.push_back(-0.01);
			setToolControl(gripper_value);
			check_mode ++;
			ROS_INFO("case 2: gripper close");
		}

	
		add_time = time(0);
		if((add_time-cur_time) >= 1)
			count ++;
			
		break;

	case 3: 
		if(check_mode == PICK_UP_MOVE_HOME)
		{
			joint_angle.push_back( 0.000 );
			joint_angle.push_back( -1.00 );
			joint_angle.push_back( 0.307 );
			joint_angle.push_back( 0.700 );
			setJointSpacePath(joint_angle, 2.0);
			check_mode ++;
			ROS_INFO("case 3: move home");	
		}
	
		add_time = time(0);
		if((add_time-cur_time) >= 6)
			count ++;
	
		break;
	

	case 4: 
		if(arrive_home == false){
			fin_pick_up.data = true;
			pick_up_pub_.publish(fin_pick_up);
			ROS_INFO("wait arrive home");
		}

		else{
			if(check_mode == MOVE_BASE && arrive_home == true)
			{
				joint_angle.push_back( 2.620 );
				joint_angle.push_back( 0.420 );
				joint_angle.push_back( -0.92 );
				joint_angle.push_back( 1.640 );
				setJointSpacePath(joint_angle, 2.0);
				check_mode ++;
				ROS_INFO("case 4: move base");
			}
		
			add_time = time(0);
			if((add_time-cur_time) >= 7){
				count ++;
				arrive_home = false;
			}
		
		}

		break;

	case 5: 
		if(check_mode == PICK_DOWN_GRIPPER_OPEN)
		{
			fin_pick_up.data = false;
			pick_up_pub_.publish(fin_pick_up);
			gripper_value.push_back(0.01);
			setToolControl(gripper_value);
			check_mode ++;
			ROS_INFO("case 5: gripper open");
		}

	
		add_time = time(0);
		if((add_time-cur_time) >= 1)
			count ++;

		break;
		

	case 6: 
		if(check_mode == PICK_DOWN_MOVE_HOME)
		{
			joint_angle.push_back( 0.000 );
			joint_angle.push_back( -1.56 );
			joint_angle.push_back( 1.100 );
			joint_angle.push_back( 0.500 );
			setJointSpacePath(joint_angle, 2.0);
			check_mode ++;
			ROS_INFO("case 6: move home");
		}

		add_time = time(0);
		if((add_time-cur_time) >= 6){
			fin_mani.data = true;
			pick_down_pub_.publish(fin_mani);
			
			if((add_time-cur_time) >= 7){
				count = 0;
				check_mode = 0;
				kinematic_pose_sub.clear();
			}
		}

		break;

	}
}

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	updateRobotState();

	if (!kinematic_pose_sub.empty())
		demoSequence();
}

void PoseCallback(const geometry_msgs::Pose &msg){

	kinematic_pose_sub.push_back(msg.position.x);
	kinematic_pose_sub.push_back(msg.position.y); 
	kinematic_pose_sub.push_back(msg.position.z);
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
	
	ros::NodeHandle nh("");
	
	ROS_INFO("11");
	ros::Subscriber sub_ = nh.subscribe("cur_mani_pose", 10, PoseCallback);
	ros::Timer publish_timer = nh.createTimer(ros::Duration(1.3), &OpenMani::publishCallback, &OpenMani);
	ros::Subscriber arrive_home_sub_ = nh.subscribe("arrive_home", 10, ArriveHomeCallback);

	pub_ = nh.advertise<geometry_msgs::Pose>("mani_pose", 10);
	pick_up_pub_ = nh.advertise<std_msgs::Bool>("fin_pick_up", 10);
	pick_down_pub_ = nh.advertise<std_msgs::Bool>("fin_mani", 10);	
	mani_plan_status_pub_ = nh.advertise<std_msgs::Bool>("mani_plan_status", 10);
	mani_error_pub_ = nh.advertise<std_msgs::Bool>("mani_error_plan", 10);
	ROS_INFO("11");

	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}


