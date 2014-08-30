#ifndef PACMAN_AGENT_H
#define PACMAN_AGENT_H

#include "agent.h"
#include "ros/ros.h"

/**
 * Abstract class that implements a pacman agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 * @extends Agent
 */
class PacmanAgent : public Agent
{
  protected:
    ros::Publisher action_publisher_;

  public:
    PacmanAgent();
    virtual void sendAction();
    std::string getAgentName();
};

#endif // PACMAN_AGENT_H