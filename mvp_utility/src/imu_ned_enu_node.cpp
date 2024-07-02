#include "ros/ros.h"
#include "mvp_utility/imu_ned_enu.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_ned_to_enu_node");

    ros::NodeHandle nh;
    IMUNedEnu node(nh);

    ros::spin();

    return 0;
}