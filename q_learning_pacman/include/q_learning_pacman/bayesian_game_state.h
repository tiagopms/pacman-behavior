#ifndef BAYESIAN_GAME_STATE_H
#define BAYESIAN_GAME_STATE_H

#include "q_learning_pacman/game_state.h"

#include "pacman_msgs/AgentPose.h"

/**
 * Abstract class that implements a pacman agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class BayesianGameState : public GameState
{
  protected:
    void observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index);
    void observePacman(int measurement_x, int measurement_y);

    ros::Subscriber pacman_pose_subscriber_;
    ros::Subscriber ghost_distance_subscriber_;

    void updatePacman(const geometry_msgs::Pose::ConstPtr& msg);
    void updateGhosts(const pacman_msgs::AgentPose::ConstPtr& msg);

  public:
    BayesianGameState();
    ~BayesianGameState();
    
    void predictPacmanMove(pacman_msgs::PacmanAction action);
    void predictGhostMove(int ghost_index);
};

#endif // BAYESIAN_GAME_STATE_H