#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>

using namespace std;

int Pick[5] = {1,3,2,5};
int Place[5] = {4,1,2,5};
int Wait[5] = {6,6,6,6};

int num; //Array number
string ppw;
bool fin_act_value;
bool fin_call;

void ppw_value(const std_msgs::String& msg){

    ppw = msg.data;
    if(ppw == "pick") ROS_INFO("pick");
    else if(ppw == "Wait") ROS_INFO("wait");
    else if(ppw == "place")  ROS_INFO("place");
    else ROS_INFO("else");
}

void finish_act_value(const std_msgs::Bool& msg){

    fin_act_value = msg.data;
}


int main(int argc, char **argv){
    ROS_INFO("main");
    
	ros::init(argc, argv,"test_turtle_mani_with_kine");
	ros::NodeHandle nh;

	ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32>("check_mode", 1);
    ros::Publisher fin_call_pubb = nh.advertise<std_msgs::Bool>("fin_call_pub", 1); ///////////////////////////////rename fin_call_pub
	ros::Publisher gripper_error_check_pub = nh.advertise<std_msgs::Int32>("gripper_error_check", 1);
	ros::Subscriber ppw_sub = nh.subscribe("pick_or_place_pub", 1, ppw_value); ///////////////////////////////rename 
    ros::Subscriber fin_act_sub = nh.subscribe("fin_act", 1, finish_act_value);

    fin_act_value = true;

    std_msgs::Bool fin_call;
    fin_call.data = true;
    std_msgs::Bool gripper_error_check_value;
    gripper_error_check_value.data = false;

    std_msgs::Int32 check_num;

	ros::Rate loop_rate(10);
    
	while(ros::ok()){

        if(ppw == "pick"){
            if(fin_act_value){
                ros::Duration(4.0).sleep(); //place 처럼 이거 아래로 내려보기
                //check_num.data = Pick[num];

                if(num >= 4){
                    //ros::Duration(1.0).sleep();
                    num = 0;

                    fin_call.data = true;
                    fin_call_pubb.publish(fin_call);
                    ROS_INFO("fin_call : %d", fin_call);

                    //ros::Duration(3.0).sleep();

                    ROS_INFO("num == 4 fin_call_pub");
                    ppw = "out";

                }
                else{
                    
                    check_num.data = Pick[num];
                    if(num == 2) ros::Duration(3.0).sleep();
                    num++;

                    fin_call.data = false;
                    fin_call_pubb.publish(fin_call);
                    ROS_INFO("fin_call : %d", fin_call);

                    chatter_pub.publish(check_num);
                    ROS_INFO("check_num : %d", check_num);
                }

                fin_act_value = false;
            }
            

        }
        if(ppw == "place"){
            if(fin_act_value){
                //ros::Duration(4.0).sleep();
                //check_num.data = Place[num];

                if(num >= 4){
                    //ros::Duration(1.0).sleep();
                    num = 0;

                    fin_call.data = true;
                    fin_call_pubb.publish(fin_call);
                    ROS_INFO("fin_call : %d", fin_call);

                    //ros::Duration(3.0).sleep();

                    ROS_INFO("num == 4 fin_call_pub");
                    ppw = "out";

                }
                else{
                    check_num.data = Place[num];
                    if(num == 1) ros::Duration(2.0).sleep();
                    
                    num++;

                    fin_call.data = false;
                    fin_call_pubb.publish(fin_call);

                    chatter_pub.publish(check_num);
                    ROS_INFO("check_num : %d", check_num);
                }

                fin_act_value = false;

                ros::Duration(4.0).sleep();
            }

        }
        if(ppw == "Wait"){
            num = 0;

            check_num.data = 6;

            chatter_pub.publish(check_num);
            ROS_INFO("check_num : %d", check_num);

            fin_call.data = false;
            fin_call_pubb.publish(fin_call);

            fin_act_value = true;

        }

        if(ppw == "wait"){
            check_num.data = 0;

            chatter_pub.publish(check_num);
            ROS_INFO("check_num : %d", check_num);

            fin_call.data = false;
            fin_call_pubb.publish(fin_call);
            
            fin_act_value = true;
        }
    
        
        ros::spinOnce();
        loop_rate.sleep();
	}
}