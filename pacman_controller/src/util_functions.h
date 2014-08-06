#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

namespace util
{
    geometry_msgs::Point createPoint(int x, int y, int z);
}