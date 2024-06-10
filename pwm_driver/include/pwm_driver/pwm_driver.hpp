#ifndef PWM_DRIVER_HPP_
#define PWM_DRIVER_HPP_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>
#include <vector>
#include "PCA9685.h"
#include <stdint.h>

class PwmDriver
{
public:
    PwmDriver(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_; 
    double m_pwm_ms_bias;
    double m_servo_pwm_ms_bias;
    
    struct thruster_t
    {
        int index;
        int channel;
        std::string topic_name;
        int min_us;
        int max_us;
        ros::Subscriber sub_;
    };
    std::vector<thruster_t> thrusters;
    std::vector<ros::Subscriber> thruster_subs_; 
    void f_thruster_callback(const std_msgs::Float64::ConstPtr& msg, int i); 

    struct servo_t
    {
        int index;
        int channel;
        int min_us;
        int max_us;
        int center_us;
        std::string topic_name;
    };
    std::vector<servo_t> servos;
    std::vector<ros::Subscriber> servo_subs_;
    void f_servo_callback(const std_msgs::Float64::ConstPtr& msg, int i);

    struct led_t
    {
        int index;
        int channel;
        std::string topic_name;
        int min_us;
        int max_us;
        ros::Subscriber sub_;
    };
    std::vector<led_t> leds;
    std::vector<ros::Subscriber> led_subs_;
    void f_led_callback(const std_msgs::Float64::ConstPtr& msg, int i); 

    PCA9685 pca{};
};

#endif