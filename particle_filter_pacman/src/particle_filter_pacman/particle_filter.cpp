#include "particle_filter_pacman/particle_filter.h"

#include "particle_filter_pacman/util_constants.h"
#include "particle_filter_pacman/util_functions.h"

#include <boost/bind.hpp>

ParticleFilter::ParticleFilter()
{
    GameParticle game_particle;
    game_particle.printMap();
    game_particles_ = std::vector< GameParticle > (util::NUMBER_OF_PARTICLES, game_particle);

    ghost_distance_subscriber_ = n_.subscribe<pacman_interface::AgentPose>("/pacman_interface/ghost_distance",5, boost::bind(&ParticleFilter::observeGhost, this, _1));
    pacman_pose_subscriber_ = n_.subscribe<geometry_msgs::Pose>("/pacman_interface/pacman_pose", 1, boost::bind(&ParticleFilter::observePacman, this, _1));
}

void ParticleFilter::estimateMovement(pacman_interface::PacmanAction action)
{
    for(std::vector< GameParticle >::reverse_iterator it = game_particles_.rbegin(); it != game_particles_.rend(); ++it)
    {
        it->move(action);
    }
}

bool compareProbabilities(std::pair< double, GameParticle > i, std::pair< double, GameParticle > j)
{
    return ( i.first < j.first );
}

bool compareProbabilities2(std::pair< double, GameParticle > i, double j)
{
    return ( i.first < j );
}

void ParticleFilter::observePacman(const geometry_msgs::Pose::ConstPtr& msg)
{
    int measurement_x = msg->position.x;
    int measurement_y = msg->position.y;

    std::map< double, GameParticle > particles_map;

    std::vector< GameParticle > new_particles;
    new_particles.reserve(util::NUMBER_OF_PARTICLES);

    double sum_prob_all_particles = 0;

    // generate a map tying particles to their probability
    for(std::vector< GameParticle >::iterator it = game_particles_.begin(); it != game_particles_.end(); ++it)
    {
        geometry_msgs::Pose pose = it->getPacmanPose();
        double probability = util::getProbOfMeasurementGivenPosition(pose.position.x, pose.position.y, measurement_x, measurement_y, 5);
        sum_prob_all_particles += probability;
        particles_map.insert(std::pair<double,GameParticle>(sum_prob_all_particles, *it));
    }

    // if no particles have probability of existing (float) show error message
    ROS_ERROR_STREAM_COND(sum_prob_all_particles == 0, "Error, all particles have a zero probability of being correct");

    double random_multiplier = sum_prob_all_particles / (double) RAND_MAX;

    // randomly select new particle group from particles map
    for (int particlesCounter = util::NUMBER_OF_PARTICLES ; particlesCounter > 0 ; particlesCounter-- )
    {
        double random_number = std::rand() * random_multiplier;

        std::map< double, GameParticle >::iterator itlow;
        itlow = particles_map.lower_bound (random_number);

        new_particles.push_back(itlow->second);
    }

    // update particles
    game_particles_.clear();
    game_particles_ = new_particles;
}

void ParticleFilter::observeGhost(const pacman_interface::AgentPose::ConstPtr& msg)
{
    
}

void ParticleFilter::printPacmanParticles()
{
    printPacmanOrGhostParticles(true, 0);
}

void ParticleFilter::printGhostParticles(int ghost_index)
{
    printPacmanOrGhostParticles(false, ghost_index);
}

void ParticleFilter::printPacmanOrGhostParticles(bool is_pacman, int ghost_index)
{
    GameParticle map = game_particles_[0];
    int height = map.getHeight();
    int width = map.getWidth();

    std::vector<float> probability_line(height, 0);
    std::vector< std::vector<float> > probability_map(width, probability_line);

    double increase_amount = 1 / (double) game_particles_.size();

    for(std::vector< GameParticle >::reverse_iterator it = game_particles_.rbegin(); it != game_particles_.rend(); ++it) {
        geometry_msgs::Pose pose = it->getPacmanPose();

        probability_map[pose.position.y][pose.position.x] += increase_amount;
    }

    for (int i = height -1 ; i > -1  ; i--) {
        std::ostringstream foo;
        foo << std::fixed;
        foo << std::setprecision(0);

        for (int j = 1 ; j < width - 1 ; j++) {
            if( map.getMapElement(j, i) == GameParticle::WALL)
                foo << "###" << ' ';
            else
            {
                int chance = probability_map[i][j]*100;

                if (chance >= 90)
                    foo << "\033[48;5;46m";
                else if (chance >= 50)
                    foo << "\033[48;5;30m";
                else if (chance >= 30)
                    foo << "\033[48;5;22m";
                else
                    foo << "\033[48;5;12m";

                foo << std::setw(3) << std::setfill('0') << chance;

                foo << "\033[0m" << ' ';
            }
        }
        ROS_INFO_STREAM(foo.str());
    }
}