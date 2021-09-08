#ifndef _GROUP_CONTROL_
#define _GROUP_CONTROL_

#include <ros/ros.h>
#include "unit_control.h"

namespace group_formation{
class group_control
{
  public:
    group_control(int c){

    }//formation

  private:
    std::vector<group_formation::group_control> units;
};
};//!namespace

#endif // !_GROUP_CONTROL_
