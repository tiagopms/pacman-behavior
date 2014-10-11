#ifndef BAYESIAN_GAME_STATE_H
#define BAYESIAN_GAME_STATE_H

#include "q_learning_pacman/game_state.h"

#include "geometry_msgs/Pose.h"
#include "pacman_msgs/AgentPoseService.h"

/**
 * Abstract class that implements a pacman agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class DeterministicGameState : public GameState
{
  protected:
    void observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index);
    void observePacman(int measurement_x, int measurement_y);
    bool observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res);

    ros::ServiceServer pacman_observer_service_;
    ros::ServiceServer ghost_distance_observer_service_;

  public:
    DeterministicGameState();
    ~DeterministicGameState();
    
    void predictPacmanMove(pacman_msgs::PacmanAction action);
    void predictGhostMove(int ghost_index);
    void predictGhostsMoves();
    void predictAgentsMoves(pacman_msgs::PacmanAction action);
};

#endif // BAYESIAN_GAME_STATE_H