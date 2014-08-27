#include "ros/ros.h"

#include "particle_filter_pacman/particle_filter.h"

#include <time.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ParticleFilter particle_filter;

    while (ros::ok())
    {
        pacman_interface::PacmanAction action;
        action.action = action.WEST;
        particle_filter.estimateMovement(action);
        particle_filter.printPacmanParticles();

     //   ROS_INFO_STREAM("Loop");

        ros::spinOnce();
        loop_rate.sleep();
    }
}