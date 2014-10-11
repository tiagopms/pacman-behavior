#include "ros/ros.h"
#include "pacman_msgs/PacmanAction.h"

#include "pacman_abstract_classes/pacman_agent.h"
#include "pacman_abstract_classes/util_functions.h"
#include "deterministic_q_learning/deterministic_game_state.h"

class DeterministicBehaviorAgent : public PacmanAgent
{
  private:
    typedef enum {STOP, EAT, EAT_BIG_FOOD, RUN, HUNT} Behaviors;

    pacman_msgs::PacmanAction getHuntAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getRunAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getEatBigFoodAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getEatAction(DeterministicGameState *game_state);
    pacman_msgs::PacmanAction getStopAction();

  public:
    DeterministicBehaviorAgent();
    pacman_msgs::PacmanAction getAction(DeterministicGameState *game_state, int behavior);
    std::string getAgentName();
};