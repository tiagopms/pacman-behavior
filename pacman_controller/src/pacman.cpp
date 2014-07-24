#include "ros/ros.h"
#include "pacman_interface/PacmanAction.h"
#include "std_msgs/String.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <string.h>

#include "agent.h"

using namespace std;

class PacmanAgent : public Agent
{
    protected:

    public:
        PacmanAgent()
        {
            action_publisher_ = n_.advertise<pacman_interface::PacmanAction>("/pacman_interface/pacman_action", 1000);
        }
        
        virtual void sendAction()
        {
            throw std::logic_error("The method or operation is not implemented.");
        }
};