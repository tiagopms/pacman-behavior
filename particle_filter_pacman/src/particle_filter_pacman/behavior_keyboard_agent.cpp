#include "particle_filter_pacman/behavior_keyboard_agent.h"

#include "ros/ros.h"
#include "particle_filter_pacman/util_constants.h"

#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

void BehaviorKeyboardAgent::keypressCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::string key = msg->data;
    boost::to_upper(key);
    if( find( this->validKeys.begin(), this->validKeys.end(), key) != this->validKeys.end() )
        keyPressed = key;
}

BehaviorKeyboardAgent::BehaviorKeyboardAgent()
{
    keypressSubscriber = n_.subscribe<std_msgs::String>("/pacman_interface/keypress", 1000, boost::bind(&BehaviorKeyboardAgent::keypressCallback, this, _1));
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);

    std::string validKeysArray[] = {"W", "E", "A", "S", "D"};
    this->validKeys.assign(validKeysArray, validKeysArray + (sizeof(validKeysArray)/sizeof(std::string)) );

    pacman_interface::PacmanAction actions;
    keyToBehavior["W"]  = STOP;
    keyToBehavior["E"]  = EAT;
    keyToBehavior["A"]  = EAT_BIG_FOOD;
    keyToBehavior["S"]  = RUN;
    keyToBehavior["D"]  = HUNT;

    this->keyPressed = "W";

    /*for(vector<string>::iterator it = this->validKeys.begin(); it != this->validKeys.end(); ++it) { 
        cout << *it << std::endl;
    }*/
    ROS_DEBUG("BehaviorKeyboardAgent initialized");
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::sendAction(ParticleFilter *particle_filter)
{
    pacman_interface::PacmanAction action;
    int behavior = keyToBehavior[this->keyPressed];
    ROS_WARN_STREAM_THROTTLE(10, "Current behavior: " << behavior);

    switch (behavior)
    {
        case STOP:
            action = getStopAction();
            break;
        case EAT:
            action = getEatAction(particle_filter);
            break;
        case EAT_BIG_FOOD:
            action = getEatBigFoodAction(particle_filter);
            break;
        case RUN:
            action = getRunAction(particle_filter);
            break;
        case HUNT:
            action = getHuntAction(particle_filter);
            break;
        default:
            action.action = action.STOP;
            break;
    }

    action_publisher_.publish(action);
    particle_filter->resetNewObservation();

    return action;
}

std::string BehaviorKeyboardAgent::getAgentName()
{
    return "BehaviorKeyboardAgent";
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getHuntAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;

    return action;
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getRunAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;

    return action;
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getEatBigFoodAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;

    return action;
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getEatAction(ParticleFilter *particle_filter) {
    pacman_interface::PacmanAction action;

    geometry_msgs::Pose pacman_pose = particle_filter->getEstimatedPacmanPose();
    std::map< std::pair<int, int>, int > distances = particle_filter->getDistances(pacman_pose.position.x, pacman_pose.position.y);

    int width = particle_filter->getMapWidth();
    int height = particle_filter->getMapHeight();
    std::vector< std::vector<GameParticle::MapElements> > map = particle_filter->getEstimatedMap();

    int min_distance = util::INFINITE;
    int food_x = -1;
    int food_y = -1;
    for (int i = 0 ; i < width ; i++)
    {
        for (int j = 0 ; j < height ; j++)
        {
            if( map[j][i] == GameParticle::FOOD)
            {
                int distance = distances[std::make_pair(i, j)];
                if(distance < min_distance)
                {
                    food_x = i;
                    food_y = j;
                    min_distance = distance;
                }
            }
        }
    }

    distances = particle_filter->getDistances(food_x, food_y);
    // TODO: add last part

    action.action = action.STOP;

    return action;
}

pacman_interface::PacmanAction BehaviorKeyboardAgent::getStopAction() {
    pacman_interface::PacmanAction action;
    action.action = action.STOP;
    return action;
}