#include "ros/ros.h"

#include "pacman_msgs/PacmanAction.h"
#include "pacman_msgs/StartGame.h"
#include "pacman_msgs/EndGame.h"

#include "q_learning_pacman/bayesian_game_state.h"

int NUMBER_OF_GAMES = 7;
int NUMBER_OF_TRAININGS = 7;

bool endGame(pacman_msgs::EndGame::Request &req, pacman_msgs::EndGame::Response &res, 
        ros::ServiceClient *start_game_client, bool *end_program, BayesianGameState **game_state)
{
    // count number of games
    static int game_count = 0;
    game_count++;

    if (req.win)
    {
        ROS_INFO_STREAM("Won game " << game_count);
    }
    else
    {
        ROS_WARN_STREAM("Lost game " << game_count);
    }

    if (game_count < NUMBER_OF_GAMES)
    {
        pacman_msgs::StartGame start_game;

        if (game_count < NUMBER_OF_TRAININGS)
            start_game.request.show_gui = false;
        else
            start_game.request.show_gui = true;

        if (start_game_client->call(start_game))
            if(start_game.response.started)
            {
                // new game started
                delete *game_state;
                *game_state = new BayesianGameState();
                res.game_restarted = true;

                ROS_INFO("Game started");
                return true;
            }
            else
                ROS_ERROR("Failed to start game (check if game already started)");
        else // if problem => print error
            ROS_ERROR("Failed to call service StartGame");
    }
    else
    {
        *end_program = true;
    }

    // game not restarted
    res.game_restarted = false;
    return true;
}

int main(int argc, char **argv)
{
    // start ros
    ros::init(argc, argv, "q_learning");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    bool end_program = false;

    BayesianGameState *game_state = new BayesianGameState();

    ros::Publisher chatter_pub = n.advertise<pacman_msgs::PacmanAction>("/pacman/pacman_action", 1000);

    // client to start game service and server for end game service
    ros::ServiceClient start_game_client = n.serviceClient<pacman_msgs::StartGame>("/pacman/start_game");
    ros::ServiceServer end_game_service = n.advertiseService<pacman_msgs::EndGame::Request, pacman_msgs::EndGame::Response>
                                ("/pacman/end_game", boost::bind(endGame, _1, _2, &start_game_client, &end_program, &game_state));
    ros::service::waitForService("/pacman/start_game", -1);

    // start first game
    pacman_msgs::StartGame start_game;
    if (NUMBER_OF_TRAININGS)
        start_game.request.show_gui = false;
    else
        start_game.request.show_gui = true;
    if (start_game_client.call(start_game))
    {
        if(start_game.response.started)
        {
            ROS_INFO("Game started");
        }
        else
        {
            ROS_ERROR("Failed to start game (check if game already started)");
        }
    }
    else // if problem print error
    {
        ROS_ERROR("Failed to call service StartGame");
    }

    while (ros::ok() && !end_program)
    {
        // print pacman probabilities map
        //game_state.printPacmanOrGhostPose(true, 0);

        // move west
        pacman_msgs::PacmanAction action;
        action.action = pacman_msgs::PacmanAction::WEST;
        chatter_pub.publish(action);
        game_state->predictGhostMove(1);
        //game_state->predictPacmanMove(action);

        // sleep and spin ros
        loop_rate.sleep();
        ros::spinOnce();
    }

    // shutdown ros node
    ros::shutdown();
}

// TODO: adicionar dois passos ou 0 para fantasmas
// TODO: adicionar dois passos e aumentar 0 para pacman
// TODO: update food probabilty
