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
#include "pid_controler.h"

#define detected 1
#define notdetected -1

using namespace std;
using namespace DJI::onboardSDK;
pid_controler* pid_c=NULL;
static float setdistance=2.0;
static float actualdistance=0;
static float read_distance=0;
static float read_x=0;
static float read_y=0;
geometry_msgs::Pose pose;
void sighandler(int signum){
	printf("Signal number:%d\n",signum);
	printf("Landing...\n");
	pid_c->exception_landing();
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
		pid_c->setflag();
	}
	else{
		printf("Tag Not Found !\n");
		pid_c->resetflag();
	}
}
int main(int argc, char **argv)
{
	int main_operate_code = 0;
	int temp32;

	ros::init(argc, argv, "dji_client");
	ROS_INFO("sdk_client_test");
	ros::NodeHandle n;
	pid_c = new pid_controler(n,setdistance,actualdistance);
	pid_c->flight_init();
	ros::Subscriber sub = n.subscribe("tag_detections_pose", 1,subscriber_distance);
	signal(SIGKILL, sighandler);
	signal(SIGQUIT, sighandler);
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGHUP, sighandler);
/*	signal(SIGSEGV,sighandler);*/
	while(ros::ok()){
		ros::spinOnce();
		
		//printf("Getting data...\n");
		if(pid_c->getflag()==notdetected){
			printf("Gimbal testing...\n");
			pid_c->gimbal_test();
		}
		else if(pid_c->getflag()==detected){
			update_pose();
			pid_c->actions(setdistance ,read_distance,read_x,read_y);
				
		}	
	}
	return 0;
}






