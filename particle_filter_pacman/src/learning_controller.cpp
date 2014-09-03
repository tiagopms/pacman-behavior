#include "ros/ros.h"

#include "particle_filter_pacman/particle_filter.h"
#include "particle_filter_pacman/learning_agent.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    LearningAgent pacman_agent;
    ParticleFilter particle_filter;

    int loop_count = 0;

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        particle_filter.estimateMap();
        //particle_filter.printMostProbableMap();
        //particle_filter.printGhostParticles(0);
        particle_filter.printGhostParticles(1);
        //particle_filter.printPacmanParticles();

        pacman_interface::PacmanAction action;
        action = pacman_agent.sendAction(&particle_filter);

        particle_filter.estimateMovement(action);
        ROS_INFO_STREAM("Loop " << loop_count);

        loop_count++;
    }
}

// TODO: Move pacman twice sometimes