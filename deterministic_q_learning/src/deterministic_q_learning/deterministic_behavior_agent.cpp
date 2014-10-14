#include "deterministic_q_learning/deterministic_behavior_agent.h"

int DeterministicBehaviorAgent::NUMBER_OF_BEHAVIORS_ = 5;

DeterministicBehaviorAgent::DeterministicBehaviorAgent()
{
    ROS_DEBUG("Behavior Agent initialized");
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getAction(DeterministicGameState *game_state, int behavior)
{
    pacman_msgs::PacmanAction action;

    switch (behavior)
    {
        case STOP:
            action = getStopAction();
            ROS_DEBUG_STREAM("Stop behavior");
            break;
        case EAT:
            action = getEatAction(game_state);
            ROS_DEBUG_STREAM("Eat behavior");
            break;
        case EAT_BIG_FOOD:
            action = getEatBigFoodAction(game_state);
            ROS_DEBUG_STREAM("Eat big food behavior");
            break;
        case RUN:
            action = getRunAction(game_state);
            ROS_DEBUG_STREAM("Run behavior");
            break;
        case HUNT:
            action = getHuntAction(game_state);
            ROS_DEBUG_STREAM("Hunt behavior");
            break;
        default:
            action.action = action.STOP;
            ROS_ERROR_STREAM("Unknown default behavior");
            break;
    }

    return action;
}

std::string DeterministicBehaviorAgent::getAgentName()
{
    return "DeterministicBehaviorAgent";
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getWestAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.WEST;

    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getEastAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.EAST;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getNorthAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.NORTH;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getSouthAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.SOUTH;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getHuntAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.EAST;

    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getRunAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.NORTH;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getEatBigFoodAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.WEST;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getEatAction(DeterministicGameState *game_state) {
    pacman_msgs::PacmanAction action;
    action.action = action.SOUTH;
    return action;
}

pacman_msgs::PacmanAction DeterministicBehaviorAgent::getStopAction() {
    pacman_msgs::PacmanAction action;
    action.action = action.STOP;
    return action;
}