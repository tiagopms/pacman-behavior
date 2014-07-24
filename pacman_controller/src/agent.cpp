#include "agent.h"

#include "pacman_interface/PacmanAction.h"

using namespace std;

Agent::Agent()
{
    action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);
}

void Agent::updatePosition()
{
    throw std::logic_error("The method or operation is not implemented.");
}

void Agent::sendAction()
{
    throw std::logic_error("The method or operation is not implemented.");
}