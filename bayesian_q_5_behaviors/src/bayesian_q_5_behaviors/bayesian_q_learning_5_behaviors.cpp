#include "bayesian_q_5_behaviors/bayesian_q_learning_5_behaviors.h"
#include "pacman_abstract_classes/util_functions.h"
#include "bayesian_q_5_behaviors/bayesian_5_behaviors_agent.h"

#include <iostream>
#include <fstream>
#include <ctime>

int BayesianQLearning::NUM_BEHAVIORS = 3;
int BayesianQLearning::NUM_FEATURES = 3;
double BayesianQLearning::learning_rate_ = 0.002;
double BayesianQLearning::discount_factor_ = 0.99;
double BayesianQLearning::exploration_rate_ = 1;
int BayesianQLearning::num_training_ = 700;
int BayesianQLearning::no_exploration_training_matches_ = 200;

BayesianQLearning::BayesianQLearning()
{
    //features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    //old_features_= std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    //weights_ = std::vector<double> (NUM_BEHAVIORS + NUM_FEATURES, 0);
    std::vector<double> temp_weights = std::vector<double> (NUM_FEATURES, 0);
    behavioral_weights_ = std::vector< std::vector<double> > (NUM_BEHAVIORS, temp_weights);
    temp_per_match_chosen_behaviors_ = std::vector<int> (NUM_BEHAVIORS, 0);
    old_q_value_ = 0;
    new_q_value_ = 0;
    old_behavior_ = 0;
    behavior_ = 0;
}

void BayesianQLearning::saveTempFeatures(int behavior)
{
    features_ = temp_features_;
    behavior_ = behavior;
}

std::vector<double> BayesianQLearning::getFeatures(BayesianGameState *game_state, int behavior)
{
    std::vector<double> features;

    // TODO: get features
    features.push_back(1.0); // bias

    

    // get distances can't be manhattan
    //features.push_back( game_state->eatsFood(action) / 10.0 );
    features.push_back( game_state->getClosestFoodDistance() / ( 1.0 * game_state->getHeight() * game_state->getWidth() ) );
    //features.push_back( game_state->getNumberOfGhostsOneStepAway(action) / 10.0 );
    //features.push_back( game_state->getNumberOfGhostsNStepsAway(3) );
    //features.push_back( 1.0 / game_state->getClosestGhostDistance() );
    features.push_back( game_state->getProbabilityOfAGhostNStepsAway(3) );
    //features.push_back( game_state->dies(action) );

    

    return features;
}

double BayesianQLearning::getQValue(BayesianGameState *game_state, int behavior)
{
    double q_value = 0;

    temp_features_ = getFeatures(game_state, behavior);
    std::vector<double>::iterator features_it = temp_features_.begin();
    //std::vector<double>::iterator weights_it = weights_.begin();
    std::vector<double> weights = behavioral_weights_[behavior];
    std::vector<double>::iterator weights_it = weights.begin();
    for(int i = 0; features_it != temp_features_.end() && weights_it != weights.end() ; ++i, ++features_it, ++weights_it)
    {
        q_value += *features_it * *weights_it;
        //ROS_INFO_STREAM(" - - - " << i << " q value " << q_value << " features_ " << *features_it << " weight " << *weights_it);
    }
    //ROS_INFO_STREAM(" - - q value " << q_value << " for behavior " << behavior);

    return q_value;
}

std::pair<int, double> BayesianQLearning::getMaxQValue(BayesianGameState *game_state)
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = - util::INFINITE;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(game_state, i);

        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
            saveTempFeatures(behavior);
        }
    }

    ROS_DEBUG_STREAM("max q behavior " << behavior << " with value " << max_q_value);

    return std::make_pair(behavior, max_q_value);
}

int BayesianQLearning::getBehavior(BayesianGameState *game_state)
{
    std::pair<int, double> behavior_q_value_pair = getMaxQValue(game_state);
    int behavior = behavior_q_value_pair.first;
    old_q_value_ = behavior_q_value_pair.second;
    return behavior;
}

int BayesianQLearning::getTrainingBehavior(BayesianGameState *game_state)
{
    double random = rand() / (float) RAND_MAX;
    if (random < exploration_rate_) {
        int behavior = rand() % NUM_BEHAVIORS;

        old_q_value_ = getQValue(game_state, behavior);
        saveTempFeatures(behavior);

        ROS_DEBUG_STREAM("random behavior " << behavior);

        return behavior;
    }
    else
        return getBehavior(game_state);
}

void BayesianQLearning::updateWeights(BayesianGameState *new_game_state, int reward)
{
    old_features_ = features_;
    old_behavior_ = behavior_;
    if (new_game_state->isFinished())
    {
        new_q_value_ = 0;
    }
    else
    {
        std::pair<int, double> behavior_q_value_pair = getMaxQValue(new_game_state);
        new_q_value_ = behavior_q_value_pair.second;
    }

    // error = reward + discount_factor * q_value(new_state) - q_value(old_state)
    double error = reward + discount_factor_ * new_q_value_ - old_q_value_;
    /*if(old_behavior_ == 1)
    {
        ROS_INFO_STREAM("updating behavior " << old_behavior_);
        ROS_ERROR_STREAM(" - error " << error);
        ROS_ERROR_STREAM(" - old q value " << old_q_value_);
        ROS_ERROR_STREAM(" - new q value " << new_q_value_);
        ROS_ERROR_STREAM(" - q value " << getQValue(new_game_state, old_behavior_));
    }
    if(old_behavior_ == 3)
    {
        ROS_INFO_STREAM("updating behavior " << old_behavior_);
        ROS_WARN_STREAM(" - error " << error);
        ROS_WARN_STREAM(" - old q value " << old_q_value_);
        ROS_WARN_STREAM(" - new q value " << new_q_value_);
        ROS_WARN_STREAM(" - q value " << getQValue(new_game_state, old_behavior_));
    }*/

    //std::vector<double>::iterator weights_it = weights_.begin();

    std::vector<double> weights = behavioral_weights_[old_behavior_];
    std::vector<double>::iterator weights_it = weights.begin();

    std::vector<double>::iterator features_it = old_features_.begin();

    for(int i = 0; features_it != old_features_.end() ; ++i, ++features_it, ++weights_it)
    {
        *weights_it = *weights_it + learning_rate_ * error * *features_it;
        /*if(old_behavior_ == 1)
        {
            ROS_ERROR_STREAM(" - feature " << *features_it);
            ROS_ERROR_STREAM(" - weight " << *weights_it);
        }
        if(old_behavior_ == 3)
        {
            ROS_WARN_STREAM(" - feature " << *features_it);
            ROS_WARN_STREAM(" - weight " << *weights_it);
        }*/
    }
    behavioral_weights_[old_behavior_] = weights;


    // TODO: remove after this
    ROS_INFO_STREAM("Error " << error << " executed behavior " << old_behavior_);
    weights = behavioral_weights_[0];
    weights_it = weights.begin();
    for(int i = 0; weights_it != weights.end() ; ++weights_it)
    {
        ROS_INFO_STREAM(" - 0 weight " << *weights_it);
    }
    weights = behavioral_weights_[1];
    weights_it = weights.begin();
    for(int i = 0; weights_it != weights.end() ; ++weights_it)
    {
        ROS_ERROR_STREAM(" - 1 weight " << *weights_it);
    }
    weights = behavioral_weights_[2];
    weights_it = weights.begin();
    for(int i = 0; weights_it != weights.end() ; ++weights_it)
    {
        ROS_WARN_STREAM(" - 2 weight " << *weights_it);
    }

    saved_behavioral_weights_.push_back(behavioral_weights_);
    saved_chosen_behaviors_.push_back(old_behavior_);
    temp_per_match_chosen_behaviors_[old_behavior_]++;

    /*if(old_behavior_ == 1)
    {
        ROS_ERROR_STREAM(" - new q value " << getQValue(new_game_state, old_behavior_));
    }
    if(old_behavior_ == 3)
    {
        ROS_WARN_STREAM(" - new q value " << getQValue(new_game_state, old_behavior_));
    }*/
}

void BayesianQLearning::saveMatchScore(int score)
{
    saved_match_scores_.push_back(score);
}

void BayesianQLearning::saveEndOfMatchWeights()
{
    saved_match_behavioral_weights_.push_back(behavioral_weights_);
    saved_per_match_chosen_behaviors_.push_back(temp_per_match_chosen_behaviors_);

    temp_per_match_chosen_behaviors_.clear();
    temp_per_match_chosen_behaviors_ = std::vector<int> (NUM_BEHAVIORS, 0);

    exploration_rate_ -= 1.0/( num_training_ - no_exploration_training_matches_ );
}

void BayesianQLearning::logWeights(std::vector< std::vector< std::vector<double> > > logged_weights, time_t time_now, bool log_per_match)
{
    // get current time to create unique file name
    struct tm *now = localtime( & time_now );
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d.%X", now);

    // output files (one for each behavior)
    std::vector< boost::shared_ptr<std::ofstream> > output_files;
    for (int i = 0; i < NUM_BEHAVIORS ; i++)
    {
        char file_name_buffer [100];
        if (log_per_match)
            sprintf(file_name_buffer, "/home/tiago/pacman_ws/log/per_match_behavior_%d__%s.txt", i, time_buf);
        else
            sprintf(file_name_buffer, "/home/tiago/pacman_ws/log/behavior_%d__%s.txt", i, time_buf);

        output_files.push_back( boost::make_shared< std::ofstream > (file_name_buffer) );

        if(! output_files[i]->is_open())
        {
            std::cout << "Unable to open file";
            return;
        }
    }

    // log weights to files
    for(std::vector< std::vector< std::vector<double> > >::iterator it = logged_weights.begin(); 
                        it != logged_weights.end(); ++it)
    {
        int file = 0;
        for(std::vector< std::vector<double> >::iterator it2 = it->begin(); 
                            it2 != it->end(); ++it2)
        {
            for(std::vector<double>::iterator it3 = it2->begin(); 
                                it3 != it2->end(); ++it3)
            {
                *output_files[file] << *it3 << " ";
            }
            *output_files[file] << "\n";
            ++file;
        }
    }

    // close output files
    for (int i = 0; i < NUM_BEHAVIORS ; i++)
    {
        output_files[i]->close();
    }
}

void BayesianQLearning::logScores(time_t time_now)
{
    // get current time to create unique file name
    struct tm *now = localtime( & time_now );
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d.%X", now);

    // output file

    char file_name_buffer [50];
    sprintf(file_name_buffer, "/home/tiago/pacman_ws/log/match_scores__%s.txt", time_buf);
    std::ofstream output_file (file_name_buffer);

    if(! output_file.is_open())
    {
        std::cout << "Unable to open file";
        return;
    }

    // log weights to files
    for(std::vector<double>::iterator it = saved_match_scores_.begin(); 
                        it != saved_match_scores_.end(); ++it)
    {
        
        output_file << *it << "\n";        
    }

    // close output file
    output_file.close();
}

void BayesianQLearning::logChosenBehaviors(time_t time_now)
{
    // get current time to create unique file name
    struct tm *now = localtime( & time_now );
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d.%X", now);

    // output file

    char file_name_buffer [50];
    sprintf(file_name_buffer, "/home/tiago/pacman_ws/log/chosen_behaviors__%s.txt", time_buf);
    std::ofstream output_file (file_name_buffer);

    if(! output_file.is_open())
    {
        std::cout << "Unable to open file";
        return;
    }

    // log weights to files
    for(std::vector<int>::iterator it = saved_chosen_behaviors_.begin(); 
                        it != saved_chosen_behaviors_.end(); ++it)
    {
        output_file << *it << "\n";        
    }

    // close output file
    output_file.close();
}

void BayesianQLearning::logEndOfMatchBehaviors(time_t time_now)
{
    // get current time to create unique file name
    struct tm *now = localtime( & time_now );
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d.%X", now);

    // output file

    char file_name_buffer [50];
    sprintf(file_name_buffer, "/home/tiago/pacman_ws/log/match_behaviors__%s.txt", time_buf);
    std::ofstream output_file (file_name_buffer);

    if(! output_file.is_open())
    {
        std::cout << "Unable to open file";
        return;
    }

    // log weights to files
    for(std::vector< std::vector<int> >::iterator it = saved_per_match_chosen_behaviors_.begin(); 
                        it != saved_per_match_chosen_behaviors_.end(); ++it)
    {
        for(std::vector<int>::iterator it2 = it->begin(); 
                            it2 != it->end(); ++it2)
        {
            output_file << *it2 << " ";
        }
        output_file << "\n";
    }

    // close output file
    output_file.close();
}

void BayesianQLearning::logWeights()
{
    std::cout << "Logging weights" << std::endl;
    
    // get current time to create unique file names
    time_t now = time(0);

    logWeights(saved_behavioral_weights_, now, false);
    logWeights(saved_match_behavioral_weights_, now, true);
    logScores(now);

    logChosenBehaviors(now);
    logEndOfMatchBehaviors(now);
}

// TODO: features ideas: closest food with min probability_of_close_enemy
// TODO:                 probability of enemy one step away
// TODO:                 weighted ghost distance, based on probabilities