#include "ros/ros.h"
#include "pwm_driver/pwm_driver.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_driver_node");
    ros::NodeHandle nh;
    PwmDriver pwm_driver(nh);
    ros::spin();
    return 0;
}
