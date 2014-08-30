#include "pacman_abstract_classes/util_functions.h"

#include "pacman_interface/PacmanAction.h"

geometry_msgs::Point util::createPoint(int x, int y, int z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

geometry_msgs::Pose util::createPose(int x, int y)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;

    return pose;
}

geometry_msgs::Pose util::sumPoses(geometry_msgs::Pose pose, geometry_msgs::Pose pose_diff)
{
    pose.position.x += pose_diff.position.x;
    pose.position.y += pose_diff.position.y;

    return pose;
}

int util::MAX_DISTANCE = 10000000;

geometry_msgs::Pose util::actionToMovement(int action)
{
    static std::map< int, geometry_msgs::Pose > actionToMovement;
    static bool initialized = false;

    if(!initialized)
    {
        actionToMovement[pacman_interface::PacmanAction::NORTH] = createPose( 0, 1);
        actionToMovement[pacman_interface::PacmanAction::SOUTH] = createPose( 0,-1);
        actionToMovement[pacman_interface::PacmanAction::EAST] = createPose( 1, 0);
        actionToMovement[pacman_interface::PacmanAction::WEST] = createPose(-1, 0);
        actionToMovement[pacman_interface::PacmanAction::STOP] = createPose( 0, 0);

        initialized = true;
    }

    return actionToMovement[action];
}