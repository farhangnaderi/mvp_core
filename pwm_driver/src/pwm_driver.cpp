#include "pwm_driver/pwm_driver.hpp"
#include <cstdio>

PwmDriver::PwmDriver(ros::NodeHandle& nh) : nh_(nh)
{
    nh_ = nh;

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

