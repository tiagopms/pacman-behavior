#ifndef GAME_STATE_H
#define GAME_STATE_H

#include "ros/ros.h"
#include <vector>

#include "pacman_msgs/PacmanAction.h"

/**
 * Class that holds information on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class GameState
{
  public:
    GameState();
    ~GameState();
    typedef enum {EMPTY, FOOD, BIG_FOOD, WALL, ERROR} MapElements;
    
    void printMap();

    int getHeight();
    int getWidth();
    MapElements getMapElement(int x, int y);

    std::vector< pacman_msgs::PacmanAction > getLegalActions(int x, int y);
    std::vector< std::pair<int, int> > getLegalNextPositions(int x, int y);
    std::vector< std::pair< float, std::pair<int, int> > > getNextPositionsForActionWithProbabilities(int x, int y, pacman_msgs::PacmanAction action);

    int getNumberOfGhosts();

    std::vector< std::vector<float> > getPacmanPoseMap();
    std::vector< std::vector<float> > getGhostPoseMap(int ghost_index);
    std::vector< std::vector< std::vector<float> > > getGhostsPoseMaps();
    std::vector< std::vector<float> > getFoodMap();
    void setPacmanPoseMap(std::vector< std::vector<float> > pacman_pose_map);
    void setGhostPoseMap(std::vector< std::vector<float> > ghost_pose_map, int ghost_index);

    static int MAX_DISTANCE;

    void printPacmanOrGhostPose( bool is_pacman, int ghost_index);

  protected:
    ros::NodeHandle n_;

    int height_;
    int width_;
    std::vector< std::vector<MapElements> > map_;

    int num_ghosts_;
    std::vector< std::vector<float> > pacman_pose_map_;
    std::vector< std::vector< std::vector<float> > > ghosts_poses_map_;
    std::vector< std::vector<float> > foods_map_;
};

#endif // GAME_STATE_H