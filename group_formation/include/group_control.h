#ifndef _GROUP_CONTROL_
#define _GROUP_CONTROL_

#include <ros/ros.h>
#include "unit_control.h"
#include <tf/transform_broadcaster.h>


namespace group_formation{
class GroupControl
{
  public:
    GroupControl(std::string group_goalTopic):group_goalTopic_(group_goalTopic){
      ros::NodeHandle nh;

      subGoal_ = nh.subscribe<geometry_msgs::Pose>(group_goalTopic_, 0, Group_Goal_Callback);

    }//formation

    void Group_Goal_Callback(geometry_msgs::Pose msg)
    {

    }

  private:
    std::vector<group_formation::UnitControl> units;

    std::string group_goalTopic_;
    ros::Subscriber subGoal_;
    ros::Subscriber subFormation_;
    
  
  


};
};//!namespace

#endif // !_GROUP_CONTROL_
