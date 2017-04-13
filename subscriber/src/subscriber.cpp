
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseArray.h"

static float read_distance=0;
void  publish_distance(float& distance){
	ros::NodeHandle n1;
	ros::Publisher distance_pub = n1.advertise<std_msgs::Float64>("Distance", 1000);
	ros::Rate loop_rate(10);
	std_msgs::Float64 msg_pub;
	msg_pub.data = read_distance;
	distance_pub.publish(msg_pub);
	ros::spin();
}
void subscriber_distance(const geometry_msgs::PoseArray& msg){

	ROS_INFO("Distance: [%f]", msg.poses[0].position.z);
	read_distance = msg.poses[0].position.z;
	publish_distance(read_distance);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle n;

	while(1){
	    ros::Subscriber sub = n.subscribe("/tag_detections_pose", 1000, subscriber_distance);
	    
	    ros::spin();
	}

	return 0;
}


