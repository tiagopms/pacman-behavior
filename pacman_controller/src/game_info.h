#ifndef GAME_H
#define GAME_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <vector>


/**
 * Class that holds information on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class GameInfo
{
  protected:
    typedef enum {EMPTY, FOOD, BIG_FOOD} MapElements;

    ros::NodeHandle n_;
    geometry_msgs::Pose pose_;

    std::vector< std::vector<MapElements> > map;

  public:
    GameInfo();
};

#endif // GAME_H