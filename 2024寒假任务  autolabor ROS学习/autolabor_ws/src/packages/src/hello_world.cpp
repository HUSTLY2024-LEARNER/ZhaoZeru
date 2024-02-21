#include "ros/ros.h"

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"hello_world");

	ros::NodeHandle nh;

	ROS_INFO("hello_world!");

	return 0;
}
