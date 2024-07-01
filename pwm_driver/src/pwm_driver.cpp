/*
 * PwmDriver.cpp
 * 
 * Author: Farhang Naderi
 * Email: farhang.naderi@uri.edu
 * License: MIT License
 * Year: 2024
 * 
 * Description:
 * This code implements the PwmDriver class, which controls the power to a servo rail and LEDs
 * based on commands received via ROS topics. It also sends heartbeat signals to an ATtiny85
 * microcontroller via I2C to ensure the system remains active and safe.
 */

#include "pwm_driver/pwm_driver.hpp"
#include <cstdio>
#include <thread>
#include <atomic>

// Include I2C libraries
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Define the I2C address of the ATtiny85
#define ATTINY85_I2C_ADDRESS 0x08

PwmDriver::PwmDriver(ros::NodeHandle& nh) : nh_(nh), running_(true)
{
    // Initialize I2C
    const char *i2c_filename = "/dev/i2c-1";
    if ((i2c_file_ = open(i2c_filename, O_RDWR)) < 0)
    {
        perror("Failed to open the i2c bus");
        return;
    }

    if (ioctl(i2c_file_, I2C_SLAVE, ATTINY85_I2C_ADDRESS) < 0)
    {
        perror("Failed to acquire bus access and/or talk to slave");
        return;
    }

    // Initialize heartbeat thread
    heartbeat_thread_ = std::thread(&PwmDriver::send_heartbeat, this);

    double m_pwm_frequency;
    nh_.param("pwm_frequency", m_pwm_frequency, 50.0);
    nh_.param("pwm_ms_bias", m_pwm_ms_bias, 0.0);

    pca.set_pwm_freq(m_pwm_frequency);

    // thruster params
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
}

PwmDriver::~PwmDriver()
{
    running_ = false;
    if (heartbeat_thread_.joinable())
    {
        heartbeat_thread_.join();
    }
    close(i2c_file_);
}

void PwmDriver::send_heartbeat()
{
    while (running_)
    {
        char heartbeat = 'H';
        if (write(i2c_file_, &heartbeat, 1) != 1)
        {
            perror("Failed to write to the i2c bus");
        }
        else
        {
            printf("Heartbeat sent\n");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void PwmDriver::f_thruster_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
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