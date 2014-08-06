#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "keyboard_agent.h"

using namespace std;

void KeyboardAgent::keypressCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    string key = msg->data;
    boost::to_upper(key);
    if( find( this->validKeys.begin(), this->validKeys.end(), key) != this->validKeys.end() )
        keyPressed = key;
}

KeyboardAgent::KeyboardAgent()
{
    keypressSubscriber = n_.subscribe<std_msgs::String>("/pacman_interface/keypress", 1000, boost::bind(&KeyboardAgent::keypressCallback, this, _1));
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);

    string validKeysArray[] = {"STOP", "WEST", "EAST", "NORTH", "SOUTH", "LEFT", "RIGHT", "UP", "DOWN"};
    this->validKeys.assign(validKeysArray, validKeysArray + (sizeof(validKeysArray)/sizeof(string)) );

    pacman_interface::PacmanAction actions;
    keyToAction["STOP"]  = actions.STOP;
    keyToAction["WEST"]  = actions.WEST; keyToAction["LEFT"] = actions.WEST;
    keyToAction["EAST"]  = actions.EAST; keyToAction["RIGHT"] = actions.EAST;
    keyToAction["NORTH"] = actions.NORTH;  keyToAction["UP"] = actions.NORTH;
    keyToAction["SOUTH"] = actions.SOUTH;  keyToAction["DOWN"] = actions.SOUTH;

    this->keyPressed = "STOP";

    /*for(vector<string>::iterator it = this->validKeys.begin(); it != this->validKeys.end(); ++it) { 
        cout << *it << std::endl;
    }*/
    ROS_DEBUG("KeyboardAgent initialized");
}

void KeyboardAgent::sendAction()
{
    pacman_interface::PacmanAction action;
    action.action = keyToAction[this->keyPressed];
    //ROS_INFO("Write heard: [%s]", this->keyPressed.c_str());
    action_publisher_.publish(action);
}

string KeyboardAgent::getAgentName()
{
    return "KeyboardAgent";
}