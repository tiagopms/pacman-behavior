#ifndef AGENT_H
#define AGENT_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

/**
 * Abstract class that implements an agent for the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class Agent
{
    protected:
        ros::NodeHandle n_;
        ros::Publisher action_publisher_;
        geometry_msgs::Pose pose_;

    public:
        Agent();
        virtual void updatePosition();
        virtual void sendAction();
};

#endif // AGENT_H