#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>

using namespace std;

int Pick[5] = {0,1,3,2,5};
int Place[5] = {0,4,1,2,5};

string ppw; 

int cnt;
bool fin_call = true;

void which(const std_msgs::String& place_or_pick){

	ppw =  place_or_pick.data;
    if(place_or_pick.data == "pick") ROS_INFO("pick");
    else if(place_or_pick.data == "Wait") ROS_INFO("wait");
    else ROS_INFO("place");
}

void fin_act_call(const std_msgs::Bool& msg){

    fin_call = msg.data;
}

int main(int argc, char **argv){
    ROS_INFO("main");
    
	ros::init(argc, argv,"what_to_do_pub");
	ros::NodeHandle nh;

	ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32>("check_mode", 100);
    ros::Publisher fin_call_pub = nh.advertise<std_msgs::Bool>("fin_call_pub", 100);
	
	ros::Subscriber sub2 = nh.subscribe("pick_or_place_pub", 10, which);
    ros::Subscriber sub3 = nh.subscribe("fin_act", 10, fin_act_call);

    std_msgs::Bool fin_act;
    fin_act.data = false;

	std_msgs::Int32 check_num;

	ros::Rate loop_rate(0.5);
    
	while(ros::ok()){
        if(ppw == "pick"){
            if(fin_call){
                //ROS_INFO("pick");
                if(cnt == 5){
                    cnt = 0;
                    fin_act.data = true;
                    ros::Duration(3.5).sleep();
                    fin_call_pub.publish(fin_act);
                    ROS_INFO("cnt == 4 fin_call_pub");
                }

                check_num.data = Pick[cnt];
                cnt++;

                ros::Duration(2.5).sleep();
        
                chatter_pub.publish(check_num);
                ROS_INFO("check_num : %d", check_num);
                fin_call = false;
            }
        
        }
        else if(ppw == "place"){
            //ROS_INFO("place");

            if(fin_call){
                if(cnt == 5){
                    cnt = 0;
                    fin_act.data = true;
                    fin_call_pub.publish(fin_act);
                    ROS_INFO("cnt == 4 fin_call_pub");
                    break;
                }
                
                check_num.data = Place[cnt];
                cnt++;

                ros::Duration(2.3).sleep();
        
                chatter_pub.publish(check_num);
                ROS_INFO("check_num : %d", check_num);
                fin_call = false;
                ROS_INFO("fin_call false : %d", fin_call);
            }
        }
        
        else if (ppw == "Wait"){
            cnt = 0;
            check_num.data = 6;

            ros::Duration(0.1).sleep();
        
            chatter_pub.publish(check_num);
            ROS_INFO("check_num : %d", check_num);

        }

        ROS_INFO("end");
        fin_act.data = false;

        ros::spinOnce();
        loop_rate.sleep();
	}
}
