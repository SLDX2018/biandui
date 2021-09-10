#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include "io/io.h"
#include "unit_control.h"
#include "proto/group_formation.pb.h"

int main(int argc, char **argv)
{
  //用于解析ROS参数，第三个参数为本节点名
  ros::init(argc, argv, "group_control");

  std::string full_path = ros::package::getPath("group_formation") + "/config/group_formation.prototxt";
  

  //实例化句柄，初始化node
  ros::NodeHandle nh;

  geometry_msgs::PoseStamped msg;

  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 1;

  msg.pose.position.x = 1;
  msg.pose.position.y = 2;
  msg.pose.position.z = 0;

  //创建publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("topic_test", 1);

  //定义发布的频率 
  ros::Rate loop_rate(1.0);
  //循环发布msg
  while (ros::ok())
  {
    ROS_INFO("Talker: GPS: x = 123321");

    
    msg.header.stamp = ros::Time::now();

    //以1Hz的频率发布msg
    pub.publish(msg);
    //根据前面定义的频率, sleep 1s
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }

  return 0;
}