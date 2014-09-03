#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"

#include "particle_filter_pacman/particle_filter.h"

class QLearning
{
  private:
    static int NUM_BEHAVIORS;

    std::vector<double> weights_;
    std::vector<double> features_;
    std::vector<double> old_features_;

    virtual std::vector<double> getFeatures(ParticleFilter *particle_filter);
    double getQValue(int action);

  public:
    QLearning();

    void updateFeatures(ParticleFilter *particle_filter);
    void updateWeights(int reward);
    int getBehavior();
};