#include "game_info.h"

#include "util_functions.h"
#include "pacman_interface/PacmanAction.h"
#include "pacman_interface/PacmanMapInfo.h"

GameInfo::GameInfo()
{
    ros::ServiceClient initInfoClient = n_.serviceClient<pacman_interface::PacmanMapInfo>("pacman_initialize_map_layout");
    pacman_interface::PacmanMapInfo initInfo;

    ros::service::waitForService("pacman_initialize_map_layout", -1);

    if (initInfoClient.call(initInfo))
    {
        std::vector<unsigned char> map = initInfo.response.layout.map;

        int width = initInfo.response.layout.width;
        int height = initInfo.response.layout.height;

        for (int i = 0 ; i < height ; i++) {
            for (int j = 0 ; j < width ; j++) {
                std::cout << map[i * width + j];
            }
                std::cout << std::endl;
        }
        std::cout << std::endl;
        
        ROS_INFO("Sum: %ld", (long int)initInfo.response.numGhosts);
    }
    else
    {
        ROS_ERROR("Failed to call service PacmanInitializationInfo");
    }
}