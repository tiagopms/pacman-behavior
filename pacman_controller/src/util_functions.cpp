#include "util_functions.h"

geometry_msgs::Point util::createPoint(int x, int y, int z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}