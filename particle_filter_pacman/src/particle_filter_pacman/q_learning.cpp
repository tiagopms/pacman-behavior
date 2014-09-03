#include "particle_filter_pacman/q_learning.h"

int QLearning::NUM_BEHAVIORS = 6;

QLearning::QLearning()
{

}

std::vector<double> QLearning::getFeatures(ParticleFilter *particle_filter)
{
    throw std::logic_error("The method getFeatures() is not implemented for base class QLearning.");
}

void QLearning::updateFeatures(ParticleFilter *particle_filter)
{
    old_features_.clear();
    old_features_ = features_;
    features_ = getFeatures(particle_filter);
}

double QLearning::getQValue(int action)
{
    double q_value = 0;

    std::vector<double>::iterator features_it = features_.begin();
    std::vector<double>::iterator weights_it = weights_.begin();
    for(; features_it != features_.end() ; ++features_it, ++weights_it)
    {
        q_value += *features_it * *weights_it;
    }

    return q_value;
}

int QLearning::getBehavior()
{
    int behavior = -1;
    double q_value = 0;
    double max_q_value = 0;

    for(int i = 0; i < NUM_BEHAVIORS ; ++i)
    {
        q_value = getQValue(i);
        if(q_value > max_q_value)
        {
            max_q_value = q_value;
            behavior = i;
        }
    }

    return behavior;
}

void QLearning::updateWeights(int reward)
{

}