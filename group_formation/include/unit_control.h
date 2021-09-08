#ifndef _UNIT_CONTROL_
#define _UNIT_CONTROL_

#include <ros/ros.h>

namespace group_formation{
class unit_control
{
  public:
    unit_control(int c){
      ros::NodeHandle nh;
      pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
      
      finalGoal.header.frame_id = "map";
    }

    geometry_msgs::PoseStamped GetFinalPosition(geometry_msgs::Pose groupGoal)
    {
      finalGoal.header.stamp = ros::Time::now();
      
      //计算实际目标姿态
      //...

      finalGoal.pose = groupGoal;
      return finalGoal;
    }

    void PublishFinalPosition(geometry_msgs::Pose groupGoal)
    {
      pub.publish<geometry_msgs::PoseStamped>(GetFinalPosition(groupGoal));
    }

  private:
    ros::Subscriber sub;
    ros::Publisher pub;

    int id;
    int relative_position;
    
    geometry_msgs::PoseStamped finalGoal;

};
};//!namespace

#endif // !unit_control
