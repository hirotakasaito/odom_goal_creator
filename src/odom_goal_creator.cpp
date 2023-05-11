#include "odom_goal_creator/odom_goal_creator.h"

OdomGoalCreator::OdomGoalCreator():private_nh("~")
{
    private_nh.param("hz",hz,{10});

    sub_odom = nh.subscribe("/t_frog/odom", 10, &OdomGoalCreator::odom_callback, this);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("/odom_goal", 1);
}

void OdomGoalCreator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    if(!received_odom) base_odom = odom;
    received_odom = true;
}

std::vector<double> OdomGoalCreator::calc_goal()
{
    if(!base_odom_set)
    {
        base_odom = odom;
        base_odom_set = true;
    }

    tf::Quaternion odom_q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Quaternion base_odom_q(base_odom.pose.pose.orientation.x, base_odom.pose.pose.orientation.y, base_odom.pose.pose.orientation.z, base_odom.pose.pose.orientation.w);
    double r,p,cy;
    double br,bp,by;
    tf::Matrix3x3(odom_q).getRPY(r, p, cy);
    tf::Matrix3x3(base_odom_q).getRPY(br, bp, by);

    geometry_msgs::PoseStamped local_goal;
    double x = odom.pose.pose.position.x - base_odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y - base_odom.pose.pose.position.y;
    std::vector<double> trans_pose{
        x*std::cos(by) + y*std::sin(by),
        -x*std::sin(by) + y*std::cos(by)
    };
    trans_pose[0] = goal[0] - trans_pose[0];
    trans_pose[1] = goal[1] - trans_pose[1];
    return trans_pose;
}

void OdomGoalCreator::input_goal()
{
    double x = 0.0;
    double y = 0.0;
    std::string a;
    printf("input goal x\n");
    std::cin>>x;
    printf("input goal y\n");
    std::cin>>y;
    goal = {x,y};
    printf("goal x: %f, goal y:%f\n", x, y);
    printf("Would you like to start? yes or no, input y/n\n");
    std::cin>>a;
    if(a=="y") received_goal = true;
}

void OdomGoalCreator::calc_reached_goal(std::vector<double> trans_pose)
{
    reached_goal = false;
    double distance = sqrt(pow(trans_pose[0], 2) + pow(trans_pose[1], 2));
    std::cout<<distance<<std::endl;
    if(distance < 0.5) reached_goal = true;
}

void OdomGoalCreator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(received_odom && received_goal)
        {
            geometry_msgs::PoseStamped local_goal;
            std::vector<double> trans_pose = calc_goal();
            calc_reached_goal(trans_pose);
            if(reached_goal)
            {
                printf("Reach Goal!!\n");
                local_goal.pose.position.x = 0.0;
                local_goal.pose.position.y = 0.0;
                received_goal = false;
                base_odom_set = false;
            }
            else
            {
                local_goal.pose.position.x = trans_pose[0];
                local_goal.pose.position.y = trans_pose[1];
            }
            local_goal.header.frame_id = "base_link";
            pub_local_goal.publish(local_goal);
        }
        else if(received_odom && !received_goal) input_goal();
        else ROS_WARN("Not received Odometry\n");

        ros::spinOnce();
        loop_rate.sleep();
   }
}
