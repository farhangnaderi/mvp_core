#include "pwm_driver/pwm_driver.hpp"
#include <cstdio>
#include <cstdlib>

PwmDriver::PwmDriver(ros::NodeHandle& nh) : nh_(nh)
{
    nh_ = nh;

    double m_pwm_frequency;
    nh_.param("pwm_frequency", m_pwm_frequency, 50.0);
    nh_.param("pwm_ms_bias", m_pwm_ms_bias, 0.0);
    nh_.param("servo_pwm_ms_bias", m_servo_pwm_ms_bias, 0.0);

    pca.set_pwm_freq(m_pwm_frequency);

    // Initialize I2C communication with ATtiny
    init_i2c();

    // Set timeout duration and initialize safety timer
    double timeout_seconds;
    nh_.param("timeout_seconds", timeout_seconds, 2.0);
    timeout_duration_ = ros::Duration(timeout_seconds);
    safety_timer_ = nh_.createTimer(ros::Duration(1.0), &PwmDriver::safety_check, this);
    last_command_time_ = ros::Time::now();

    // thruster parms
    int m_thruster_num;
    std::vector<int> m_thruster_ch_list;
    std::vector<std::string> m_thruster_topic_list;
    std::vector<int> m_thruster_min_us;
    std::vector<int> m_thruster_max_us;
    std::vector<int> m_thruster_init_us;

    nh_.param("thruster_num", m_thruster_num, 8);
    nh_.getParam("thruster_ch_list", m_thruster_ch_list);
    nh_.getParam("thruster_topic_list", m_thruster_topic_list);
    nh_.getParam("thruster_min_us", m_thruster_min_us);
    nh_.getParam("thruster_max_us", m_thruster_max_us);
    nh_.getParam("thruster_init_us", m_thruster_init_us);

    // led params
    std::vector<int> m_led_ch_list;
    std::vector<std::string> m_led_topic_list;
    std::vector<int> m_led_min_us;
    std::vector<int> m_led_max_us;
    std::vector<int> m_led_init_us;

    nh_.getParam("led_ch_list", m_led_ch_list);
    nh_.getParam("led_topic_list", m_led_topic_list);
    nh_.getParam("led_min_us", m_led_min_us);
    nh_.getParam("led_max_us", m_led_max_us);
    nh_.getParam("led_init_us", m_led_init_us);

    // servo params
    std::vector<int> m_servo_ch_list;
    std::vector<std::string> m_servo_topic_list;
    std::vector<int> m_servo_min_us;
    std::vector<int> m_servo_max_us;
    std::vector<int> m_servo_center_us;

    nh_.getParam("servo_ch_list", m_servo_ch_list);
    nh_.getParam("servo_topic_list", m_servo_topic_list);
    nh_.getParam("servo_min_us", m_servo_min_us);
    nh_.getParam("servo_max_us", m_servo_max_us);
    nh_.getParam("servo_center_us", m_servo_center_us);

    // declare subscriptions for thrusters
    for (int i = 0; i < m_thruster_ch_list.size(); i++)
    {
        thruster_t t;
        t.index = i;
        t.channel = m_thruster_ch_list[i];
        t.topic_name = m_thruster_topic_list[i];
        t.min_us = m_thruster_min_us[i];
        t.max_us = m_thruster_max_us[i];
        thruster_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_thruster_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, m_thruster_init_us[i] / 1000.0 + m_pwm_ms_bias);
        thrusters.push_back(t);
    }

    // declare subscriptions for LEDs
    for (int i = 0; i < m_led_ch_list.size(); i++)
    {
        led_t t;
        t.index = i;
        t.channel = m_led_ch_list[i];
        t.topic_name = m_led_topic_list[i];
        t.min_us = m_led_min_us[i];
        t.max_us = m_led_max_us[i];
        led_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_led_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, m_led_init_us[i] / 1000.0 + m_pwm_ms_bias);
        leds.push_back(t);
    }

    // declare subscriptions for servos
    for (int i = 0; i < m_servo_ch_list.size(); i++)
    {
        servo_t t;
        t.index = i;
        t.channel = m_servo_ch_list[i];
        t.topic_name = m_servo_topic_list[i];
        t.min_us = m_servo_min_us[i];
        t.max_us = m_servo_max_us[i];
        t.center_us = m_servo_center_us[i];
        servo_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_servo_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, t.center_us / 1000.0 + m_servo_pwm_ms_bias);
        servos.push_back(t);
    }
}

void PwmDriver::init_i2c()
{
    attiny_fd = wiringPiI2CSetup(attiny_i2c_address);
    if (attiny_fd == -1) {
        ROS_ERROR("Failed to initialize I2C communication with ATtiny");
    }
}

void PwmDriver::f_thruster_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Update the last command time
    last_command_time_ = ros::Time::now();

    // Send working state to ATtiny
    send_i2c_data(true);

    // scale it
    if (msg->data >= -1.0 && msg->data <= 1.0)
    {
        float a = (thrusters[i].max_us - thrusters[i].min_us) / 2.0;
        float b = (thrusters[i].max_us + thrusters[i].min_us) / 2.0;
        double u = (a * msg->data + b) / 1000.0 + m_pwm_ms_bias;
        printf("ch=%d, pwm=%lf\r\n", thrusters[i].channel, u - m_pwm_ms_bias);
        pca.set_pwm_ms(thrusters[i].channel, u);
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::f_led_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Update the last command time
    last_command_time_ = ros::Time::now();

    // Send working state to ATtiny
    send_i2c_data(true);

    // scale it
    if (msg->data >= 0.0 && msg->data <= 1.0)
    {
        float a = (leds[i].max_us - leds[i].min_us);
        float b = leds[i].min_us;
        double u = (a * msg->data + b) / 1000.0 + m_pwm_ms_bias;
        printf("ch=%d, pwm=%lf\r\n", leds[i].channel, u - m_pwm_ms_bias);
        pca.set_pwm_ms(leds[i].channel, u);
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::f_servo_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Update the last command time
    last_command_time_ = ros::Time::now();

    // Send working state to ATtiny
    send_i2c_data(true);

    // scale it
    if (msg->data >= -1.0 && msg->data <= 1.0)
    {
        float a = (servos[i].max_us - servos[i].min_us) / 2.0;
        float b = (servos[i].max_us + servos[i].min_us) / 2.0;
        double u = (a * msg->data + b) / 1000.0 + m_servo_pwm_ms_bias;
        printf("ch=%d, pwm=%lf\r\n", servos[i].channel, u - m_servo_pwm_ms_bias);
        pca.set_pwm_ms(servos[i].channel, u);
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::safety_check(const ros::TimerEvent& event)
{
    // Check if the last command was received within the timeout duration
    if ((ros::Time::now() - last_command_time_) > timeout_duration_)
    {
        // Timeout occurred, kill PWM signals
        for (auto& servo : servos)
        {
            pca.set_pwm(servo.channel, 0, 0);  // Set PWM to zero (off)
        }

        // Send not working state to ATtiny
        send_i2c_data(false);
    }
}

void PwmDriver::send_i2c_data(bool working)
{
    if (attiny_fd != -1) { // Check if the I2C file descriptor is valid
        // Generate a random number to indicate the system state
        // If 'working' is true, generate a number between 0 and 255
        // If 'working' is false, generate a number between 256 and 511
        int data = working ? (rand() % 256) : (rand() % 256 + 256);

        // Send the generated random number to the ATtiny via I2C
        wiringPiI2CWrite(attiny_fd, data);

        // Print the sent data for debugging purposes
        printf("Sent I2C data: %d\n", data);
    } else {
        // If the I2C file descriptor is invalid, print an error message
        printf("Failed to send I2C data: Invalid file descriptor\n");
    }
}