#include "mvp_utility/pressure_to_depth.hpp"

PressureToDepthNode::PressureToDepthNode(ros::NodeHandle& nh, std::string name)
{
    //parameter
    nh.param("fluid_density", m_fluid_density, 1023.0f);
    nh.param("frame_id", m_frame_id, std::string("world"));

    //topic with namespace
    _publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("depth", 10);
    //topic without namespace
    _subscriber = nh.subscribe("pressure", 10, &PressureToDepthNode::f_pressure_callback, this);
}

void PressureToDepthNode::f_pressure_callback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped depth;
    depth.header = msg->header;
    depth.header.frame_id = m_frame_id;
    depth.pose.pose.position.z = msg->fluid_pressure / (m_fluid_density * 9.81);
    
    _publisher.publish(depth);
}