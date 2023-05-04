#ifndef ODOM_GOAL_CREATOR_H
#define ODOM_GOAL_CREATOR_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

class OdomGoalCreator
{
public:
    OdomGoalCreator();
    void process();

private:

    void odom_callback(const nav_msgs::Odometry::ConstPtr &);
    std::vector<double> calc_goal();
    void input_goal();
    bool calc_reached_goal(std::vector<double>);

    bool received_odom = false;
    bool received_goal = false;
    int hz;
    std::vector<double> goal;

    ros::Subscriber sub_odom;
    ros::Publisher pub_local_goal;
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry base_odom;
};

#endif
