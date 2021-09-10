#ifndef _UNIT_CONTROL_
#define _UNIT_CONTROL_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace group_formation{
class UnitControl
{
  public:
    UnitControl(std::string cmd_velTopic, tf::Transform relativePosition): cmd_velTopic_(cmd_velTopic),
                                                                           relativePosition_(relativePosition)
    {
      ros::NodeHandle nh;
      pub_ = nh.advertise<geometry_msgs::PoseStamped>(cmd_velTopic_, 1);
      
      finalGoal_.header.frame_id = "map";
    }

    geometry_msgs::PoseStamped SetFinalPosition(geometry_msgs::Pose groupGoal)
    {
      finalGoal_.header.stamp = ros::Time::now();
      
      //计算实际目标姿态
      //...

      finalGoal_.pose = groupGoal;
      return finalGoal_;
    }

    void PublishFinalPosition(geometry_msgs::Pose groupGoal)
    {
      pub_.publish<geometry_msgs::PoseStamped>(SetFinalPosition(groupGoal));
    }

  private:
    ros::Publisher pub_;

    std::string cmd_velTopic_;
    tf::Transform relativePosition_;
    
    geometry_msgs::PoseStamped finalGoal_;

};
};//!namespace

#endif // !unit_control
