#include "deterministic_q_learning/deterministic_q_learning.h"
#include "pacman_abstract_classes/util_functions.h"

int DeterministicQLearning::NUM_BEHAVIORS = 5;
int DeterministicQLearning::NUM_FEATURES = 7;
double DeterministicQLearning::learning_rate_ = 0.5;
double DeterministicQLearning::discount_factor_ = 0.95;
double DeterministicQLearning::exploration_rate_ = 0.5;
int DeterministicQLearning::num_training_ = 10;

DeterministicQLearning::DeterministicQLearning()
{
    features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

std::vector<double> DeterministicQLearning::getFeatures(DeterministicGameState *game_state)
{
    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        features_[i] = 0;
    }

    // TODO: get features

    return features_;
}

double DeterministicQLearning::getQValue(DeterministicGameState *game_state, int behavior)
{
    double q_value = 0;

    features_ = getFeatures(game_state);
    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(int i = 0; features_it != features_.end() && weights_it != weights_.end() ; ++i, ++features_it, ++weights_it)
    {
        if(i == behavior)
        {
            q_value += 1 * *weights_it;
        }
        else
        {
            q_value += *features_it * *weights_it;
        }
        //ROS_INFO_STREAM(" - - - q value " << q_value << " features_ " << *features_it << " weight " << *weights_it);
    }
    //ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

std::pair<int, double> DeterministicQLearning::getMaxQValue(DeterministicGameState *game_state)
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = - util::INFINITE;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(game_state, i);
    //    ROS_INFO_STREAM(" - - q value " << q_value);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }

    return std::make_pair(behavior, max_q_value);
}

int DeterministicQLearning::getBehavior(DeterministicGameState *game_state)
{
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(game_state);
    int behavior = behavior_q_value_pair.first;
    old_q_value_ = behavior_q_value_pair.second;
    return behavior;
}

void DeterministicQLearning::updateWeights(DeterministicGameState *new_game_state, int reward)
{
    old_features_ = features_;
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(new_game_state);

    // error = reward + discount_factor * q_value(new_state) - q_value(old_state)
    double error = reward + discount_factor_ * new_q_value_ - old_q_value_;
    //ROS_INFO_STREAM(" - error " << error);

    std::vector<double>::iterator weights_it = weights_.begin();
    std::vector<double>::iterator features_it = old_features_.begin();

    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
    //    ROS_INFO_STREAM(" - feature " << *features_it);
    //    ROS_INFO_STREAM(" - weight " << *weights_it);
    }
}

// TODO: features ideas: closest food with min probability_of_close_enemy
// TODO:                 probability of enemy one step away
// TODO:                 weighted ghost distance, based on probabilities