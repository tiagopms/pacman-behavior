#include "deterministic_q_learning/deterministic_game_state.h"

#include "pacman_abstract_classes/util_functions.h"


DeterministicGameState::DeterministicGameState()
{
    pacman_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/pacman_pose", boost::bind(&DeterministicGameState::observeAgent, this, _1, _2));
    ghost_distance_observer_service_ = n_.advertiseService<pacman_msgs::AgentPoseService::Request, pacman_msgs::AgentPoseService::Response>
                                ("/pacman/ghost_distance", boost::bind(&DeterministicGameState::observeAgent, this, _1, _2));

    ROS_DEBUG_STREAM("Bayesian game state initialized");
}

DeterministicGameState::~DeterministicGameState()
{
        pacman_observer_service_.shutdown();
        ghost_distance_observer_service_.shutdown();

        ROS_DEBUG_STREAM("Bayesian game state destroyed");
}

void DeterministicGameState::observePacman(int measurement_x, int measurement_y)
{
    pacman_pose_.position.x = measurement_x;
    pacman_pose_.position.y = measurement_y;
}

void DeterministicGameState::observeGhost(int measurement_x_dist, int measurement_y_dist, int ghost_index)
{
    ghosts_poses_[ghost_index].position.x = pacman_pose_.position.x + measurement_x_dist;
    ghosts_poses_[ghost_index].position.y = pacman_pose_.position.y + measurement_y_dist;
}

bool DeterministicGameState::observeAgent(pacman_msgs::AgentPoseService::Request &req, pacman_msgs::AgentPoseService::Response &res)
{
    int agent = (int) req.agent;
    geometry_msgs::Pose pose = (geometry_msgs::Pose) req.pose;
    int measurement_x = pose.position.x;
    int measurement_y = pose.position.y;

    if(agent == pacman_msgs::AgentPoseService::Request::PACMAN)
    {
        ROS_DEBUG_STREAM("Observe pacman");
        observePacman(measurement_x, measurement_y);
    }
    else
    {
        int ghost_index = agent - 1;
        ROS_DEBUG_STREAM("Observe ghost " << ghost_index);
        observeGhost(measurement_x, measurement_y, ghost_index);
    }

    res.observed = true;
    return true;
}

void DeterministicGameState::predictPacmanMove(pacman_msgs::PacmanAction action)
{
    ROS_DEBUG_STREAM("Predict pacman");

    // TODO: add things here
}

void DeterministicGameState::predictGhostMove(int ghost_index)
{
    ROS_DEBUG_STREAM("Predict ghost " << ghost_index);

    // TODO: add things here
}

void DeterministicGameState::predictGhostsMoves()
{
    for(int i = 0 ; i < num_ghosts_ ; ++i)
    {
        predictGhostMove(i);
    }
}

void DeterministicGameState::predictAgentsMoves(pacman_msgs::PacmanAction action)
{
    predictPacmanMove(action);
    predictGhostsMoves();
}