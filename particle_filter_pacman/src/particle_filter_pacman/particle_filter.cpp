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

    std::vector< double > probabilities;
    probabilities.reserve(util::NUMBER_OF_PARTICLES);

    std::map< double, GameParticle > particles_map;
    std::vector< std::pair< double, GameParticle > > particles_pair;
    particles_pair.reserve(util::NUMBER_OF_PARTICLES);

    std::vector< GameParticle > new_particles;
    new_particles.reserve(util::NUMBER_OF_PARTICLES);

const clock_t begin_time = clock();

    double sum_prob_all_particles = 0;

    for(std::vector< GameParticle >::iterator it = game_particles_.begin(); it != game_particles_.end(); ++it)
    {
        geometry_msgs::Pose pose = it->getPacmanPose();
        double probability = util::getProbOfMeasurementGivenPosition(pose.position.x, pose.position.y, measurement_x, measurement_y, 5);
       // probabilities.push_back(probability);
        sum_prob_all_particles += probability;

        particles_pair.push_back(std::pair<double,GameParticle>(sum_prob_all_particles, *it));
    }

    ROS_ERROR_STREAM_COND(sum_prob_all_particles == 0, "Error, all particles have a zero probability of being correct");
    ROS_ERROR_STREAM("Probs: " << sum_prob_all_particles);
    ROS_ERROR_STREAM("X: " << measurement_x << " Y: " << measurement_y);

    std::sort (particles_pair.begin(), particles_pair.end(), compareProbabilities);

    // select new particle group
    int particlesCounter = util::NUMBER_OF_PARTICLES;
    double sum_probs = 0;
    double random_multiplier = sum_prob_all_particles / (double) RAND_MAX;

    while (particlesCounter > 0)
    {
        double random_number = std::rand() * random_multiplier;


        std::vector< std::pair< double, GameParticle > >::iterator low;
        low=std::lower_bound (particles_pair.begin(), particles_pair.end(), random_number, compareProbabilities2);

        particlesCounter--;
        new_particles.push_back(low->second);
    }

static float mean = 0;
static int number_of_it = 1;
mean = mean * ( ( number_of_it - 1 ) / (double) number_of_it) + (float( clock () - begin_time )  /  CLOCKS_PER_SEC) * ( ( 1 ) / (double) number_of_it);
number_of_it++;
std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
std::cout << mean << std::endl;
    /*while (particlesCounter > 0)
    {
       // ROS_INFO_STREAM_THROTTLE(1, "Selecting particle " << sum_probs);
      //  ROS_INFO_STREAM_THROTTLE(1, "Random " << random_variable);
       // ROS_WARN_STREAM("Selecting particle " << sum_probs);
        int counter = util::NUMBER_OF_PARTICLES - 1;
        for(std::vector< double >::reverse_iterator it = probabilities.rbegin(); it != probabilities.rend(); ++it)
        {
            sum_probs += (double) *it;
            while(sum_probs >= random_variable)
            {
                particlesCounter--;
                new_particles.push_back(game_particles_[counter]);
                random_variable += ( std::rand() / (double) RAND_MAX ) * sum_prob_all_particles;

                if(particlesCounter == 0)
                    goto AFTER_PACMAN_PARTICLES_SELECTED;
            }
            counter--;
        }
    }*/

    AFTER_PACMAN_PARTICLES_SELECTED:

    ROS_INFO_STREAM("New observation " << particlesCounter);
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