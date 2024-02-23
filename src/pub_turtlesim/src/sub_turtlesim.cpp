#include "ros/ros.h"
#include "turtlesim/Pose.h"

void readPose(const turtlesim::Pose::ConstPtr& p)
{
    ROS_INFO("位姿: x=%.2f, y=%.2f, theta=%.2f, lv=%.2f, av=%.2f",
        p->x, p->y, p->theta, p->linear_velocity, p->angular_velocity);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc,argv,"sub_turtlesim");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,readPose);

    ros::spin();
    
    return 0;
}