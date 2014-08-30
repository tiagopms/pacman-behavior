#include "ros/ros.h"

#include "particle_filter_pacman/particle_filter.h"
#include "particle_filter_pacman/behavior_keyboard_agent.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    BehaviorKeyboardAgent pacman_agent;
    ParticleFilter particle_filter;

    int loop_count = 0;

    while (ros::ok())
    {
//        particle_filter.printGhostParticles(1);
        loop_rate.sleep();
        ros::spinOnce();
        /*while( !particle_filter.hasNewObservation() && ros::ok())
        {
            ROS_INFO_STREAM_THROTTLE(1, "Waiting");
            ros::spinOnce();
            loop_rate.sleep();
        }*/

        pacman_interface::PacmanAction action;
        action = pacman_agent.sendAction(&particle_filter);

        particle_filter.estimateMovement(action);
        ROS_INFO_STREAM("Loop " << loop_count);

        loop_count++;
    }
}