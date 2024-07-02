/*
    This node converts IMU readings from NED frame to ENU frame
*/

#ifndef IMU_NED_ENU_HPP_
#define IMU_NED_ENU_HPP_

#include "ros/ros.h"
#include <string>
#include "sensor_msgs/Imu.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/FluidPressure.h"

class IMUNedEnu
{
    public:
        IMUNedEnu(ros::NodeHandle& nh, std::string name = "imu_ned_to_enu");
    
    private:
        std::string m_frame_id;

        double roll_offset;
        double pitch_offset;
        double yaw_offset;
        
        double roll_reverse;
        double pitch_reverse;
        double yaw_reverse;

        void f_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);        
        
        ros::Publisher m_imu_out;
        ros::Subscriber m_imu_in;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif