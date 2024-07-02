#include "ros/ros.h"
#include "mvp_utility/pressure_to_depth.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pressure_to_depth_node");

    ros::NodeHandle nh;
    PressureToDepthNode node(nh);

    ros::spin();

    return 0;
}