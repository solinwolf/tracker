//launch-prefix="xterm -e gdb --args"

#include <ros/ros.h>
#include <stdio.h>

#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <signal.h>
#include "controler.h"
#include <stdlib.h>
//#include <dji_sdk/dji_drone.h>
#define detected 1
#define notdetected -1
int takeoff=0;
using namespace std;
using namespace DJI::onboardSDK;
controler* con_c=NULL;
static float setdistance=1.5;
static float actualdistance=0;
static float read_distance=0;
static float read_x=0;
static float read_y=0;
geometry_msgs::Pose pose;
void sighandler(int signum){
	printf("Signal number:%d\n",signum);
	printf("Landing...\n");
	con_c->exception_landing();
	exit(-11);
}

void update_pose(){
        read_distance = pose.position.z;
	read_x = pose.position.x;
	read_y = pose.position.y;
}
void subscriber_distance(const geometry_msgs::PoseArray::ConstPtr& msg){

	if(msg->poses.size()>0){
		pose = msg->poses.front();
		printf("Received distance:%f\n",pose.position.z);
		update_pose();
		con_c->setflag();
	}
	else{
		printf("Tag Not Found !\n");
		con_c->resetflag();
	}
}

void get_data_from_mobile_callback(dji_sdk::TransparentTransmissionData data)
{
    printf("size of data = %d\n",data.data.size());
    if(data.data.size()>0){
	if(data.data[0]==67 && data.data[1]==100){
		//takeoff
		takeoff=1;
		con_c->flight_init();
		printf("Taking off\n");
	}
	else if(data.data[0]==67 && data.data[1]==101){
		//Landing
		takeoff=-1;
		printf("Landing\n");
	}
	
    }
	

}
int main(int argc, char **argv)
{
	int main_operate_code = 0;
	int temp32;

	ros::init(argc, argv, "dji_client");
	ROS_INFO("sdk_client_test");
	ros::NodeHandle n;
	con_c = new controler(n,setdistance,actualdistance);
	
	ros::Subscriber sub = n.subscribe("tag_detections_pose", 1,subscriber_distance);
	ros::Subscriber mobile_data_subscriber = n.subscribe<dji_sdk::TransparentTransmissionData>("dji_sdk/data_received_from_remote_device",2,get_data_from_mobile_callback);
	signal(SIGKILL, sighandler);
	signal(SIGQUIT, sighandler);
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGHUP, sighandler);
	signal(SIGSEGV,sighandler);
	while(ros::ok()){
		ros::spinOnce();
		if(takeoff==1){
			if(con_c->getflag()==notdetected){
				printf("Gimbal testing...\n");
				con_c->gimbal_test();
			}
			else if(con_c->getflag()==detected){
			
				con_c->actions(setdistance ,read_distance,read_x,read_y);
				
			}
		}
		else if(takeoff==-1){
			printf("exception_landing\n");
			con_c->exception_landing();
		}	
	}
	return 0;
}






