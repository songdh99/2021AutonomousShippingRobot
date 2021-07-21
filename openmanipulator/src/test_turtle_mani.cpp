#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

#include <sstream>


void PoseCallback(const geometry_msgs::Pose &msg)
{

	ROS_INFO("%f", msg.position.x);
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");

	

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("aruco_xyz", 10, PoseCallback);
	
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("fin_pickup", 10);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		std_msgs::Bool bool_msg;
		bool_msg.data = true;
		ROS_INFO("%d",bool_msg.data);
		chatter_pub.publish(bool_msg);
		ros::spinOnce();

		loop_rate.sleep();
	}

	
}
