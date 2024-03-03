#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //调用transform必须包含该头文件

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "tf_sub");

    ros::NodeHandle nh;
    
    //创建TF订阅节点
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while(ros::ok())
    {
        //生成座标点（相对于子级坐标系）
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser" ;
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 3;

        //转换坐标系（相对于父坐标系）
        try //使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");//baser_link
            ROS_INFO("转换后：(%.2f, %.2f, %.2f), 参考：%s",point_base.point.x,point_base.point.y,point_base.point.z,point_base.header.frame_id.c_str());//point_base.header.frame_id.c_str()
        }
        catch(const std::exception& e)
        {
            ROS_INFO("程序异常");
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;

}

