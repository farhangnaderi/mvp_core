#include "mvp_utility/imu_ned_enu.hpp"
#include "Eigen/Dense"
#include "iostream"
#include "cstdio"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include <functional>
#include <memory>

IMUNedEnu::IMUNedEnu(ros::NodeHandle& nh, std::string name)
{
    //parameter
    nh.param("roll_offset", roll_offset, 3.1415926);
    nh.param("pitch_offset", pitch_offset, 0.0);
    nh.param("yaw_offset", yaw_offset, 1.5707);
    nh.param("roll_reverse", roll_reverse, 1.0);
    nh.param("pitch_reverse", pitch_reverse, -1.0);
    nh.param("yaw_reverse", yaw_reverse, -1.0);
    nh.param("frame_id", m_frame_id, std::string("imu"));

    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //topic with namespace
    m_imu_out = nh.advertise<sensor_msgs::Imu>("imu_out/data", 10);
    //topic without namespace
    m_imu_in = nh.subscribe("imu_in/data", 10, &IMUNedEnu::f_imu_callback, this);
}

void IMUNedEnu::f_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // geometry_msgs::TransformStamped t;
    // ros::Time now = ros::Time::now();

    // std::string fromFrameRel = "mvp2_test_robot/cg_link";
    // std::string toFrameRel = "mvp2_test_robot/world_ned";

    // try {
    //     t = tf_buffer_->lookupTransform(
    //     toFrameRel, fromFrameRel,
    //     ros::Time(0),
    //     ros::Duration(0.01)
    //     );
    // } catch (const tf2::TransformException & ex) {
    //     ROS_INFO("Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //     return;
    // }

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion newq;
    // newq.setRPY(roll-M_PI, -pitch, -yaw + M_PI_2); default?
    // newq.setRPY(roll, -pitch, -yaw + M_PI_2);
    newq.setRPY(roll * roll_reverse + roll_offset, 
                pitch * pitch_reverse + pitch_offset,
                yaw * yaw_reverse + yaw_offset);

    newq = newq.normalize();

    sensor_msgs::Imu m = *msg;
 
    m.orientation.x = newq.x();
    m.orientation.y = newq.y();
    m.orientation.z = newq.z();
    m.orientation.w = newq.w();
    m.header.frame_id = m_frame_id;
    m_imu_out.publish(m);
}