#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "ros/ros.h"
#include <vector>

#include "particle_filter_pacman/game_particle.h"
#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/AgentPose.h"
#include "geometry_msgs/Pose.h"

/**
 * Class that implements a particle filter on the pacman game.
 * 
 * @author Tiago Pimentel Martins da Silva
 */
class ParticleFilter
{
  public:
    ParticleFilter();

    void estimateMovement(pacman_interface::PacmanAction action);

    void printPacmanParticles();
    void printGhostParticles(int ghost_index);

  protected:
    ros::NodeHandle n_;
    ros::Subscriber ghost_distance_subscriber_;
    ros::Subscriber pacman_pose_subscriber_;
    std::vector< GameParticle > game_particles_;

    void observePacman(const geometry_msgs::Pose::ConstPtr& msg);
    void observeGhost(const pacman_interface::AgentPose::ConstPtr& msg);

    void printPacmanOrGhostParticles(bool is_pacman, int ghost_index);
};

#endif // PARTICLE_FILTER_H