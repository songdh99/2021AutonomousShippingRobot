#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>

#include <sstream>


void PoseCallback(const std_msgs::Int32& num)
{

	ROS_INFO(" mode_number : %d", num.data);
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("mode", 10, PoseCallback);
	
	
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("mani_pos", 10);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		geometry_msgs::Pose msg;
		
		msg.position.x = 0.2;
		msg.position.y = 0.2;
		msg.position.z = 0.2;
		chatter_pub.publish(msg);


		ros::spinOnce();

		loop_rate.sleep();
	}

	
}
