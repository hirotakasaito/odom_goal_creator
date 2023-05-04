#include <odom_goal_creator/odom_goal_creator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_goal_creator");
    OdomGoalCreator odom_goal_creator;
    odom_goal_creator.process();
    return 0;
}
