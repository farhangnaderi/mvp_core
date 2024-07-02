/*
    This node converts pressure reading into pose with covariance stamped message for robot localization
*/

#ifndef PRESSURE_TO_DEPTH_HPP_
#define PRESSURE_TO_DEPTH_HPP_

#include "ros/ros.h"
#include <string>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/FluidPressure.h"

class PressureToDepthNode
{
    public:
        PressureToDepthNode(ros::NodeHandle& nh, std::string name = "pressure_to_depth_node");
    
    private:
        ros::Publisher _publisher;
        ros::Subscriber _subscriber;
        void f_pressure_callback(const sensor_msgs::FluidPressure::ConstPtr& msg);
        std::string m_frame_id;
        float m_fluid_density;
};

#endif