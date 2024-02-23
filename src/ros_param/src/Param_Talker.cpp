#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Param_Talker");

    std::vector<std::string> stus;
    stus.push_back("张三");
    stus.push_back("李四");
    stus.push_back("弦五");
    stus.push_back("zzr");

    std::map<std::string,std::string> friends;
    friends["zhao"] = "gao";
    friends["sun"] = "ding";

    ros::NodeHandle nh;
    nh.setParam("nh_int",10);
    nh.setParam("nh_double",3.14);
    nh.setParam("nh_bool",true); //bool
    nh.setParam("nh_string","hello NodeHandle"); //字符串
    nh.setParam("nh_vector",stus); // vector
    nh.setParam("nh_map",friends); // map

    //修改演示(相同的键，不同的值)
    nh.setParam("nh_int",10000);

    //param--------------------------------------------------------
    ros::param::set("param_int",20);
    ros::param::set("param_double",3.14);
    ros::param::set("param_string","Hello Param");
    ros::param::set("param_bool",false);
    ros::param::set("param_vector",stus);
    ros::param::set("param_map",friends);

    //修改演示(相同的键，不同的值)
    ros::param::set("param_int",20000);

    return 0;

}