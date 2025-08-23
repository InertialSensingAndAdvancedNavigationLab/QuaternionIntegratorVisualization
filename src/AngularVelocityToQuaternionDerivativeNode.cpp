/**
 * @file AngularVelocityToQuaternionDerivativeNode.cpp
 * @brief Implementation of the AngularVelocityToQuaternionDerivativeNode class.
 * @author Gemini
 * @date 2025-08-23
 */
#include "quaternion_integrator/AngularVelocityToQuaternionDerivativeNode.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For converting geometry_msgs::Quaternion to tf2::Quaternion

namespace quaternion_integrator
{

AngularVelocityToQuaternionDerivativeNode::AngularVelocityToQuaternionDerivativeNode(ros::NodeHandle& nh)
    : nh_(nh)
    , has_received_orientation_(false)
{
    // Initialize subscribers
    imu_sub_ = nh_.subscribe("imu/data_corrected", 10, &AngularVelocityToQuaternionDerivativeNode::imuCallback, this);
    orientation_sub_ = nh_.subscribe("imu/integrated_orientation", 10, &AngularVelocityToQuaternionDerivativeNode::orientationCallback, this);

    // Initialize publisher
    quaternion_derivative_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("imu/quaternion_derivative", 10);

    ROS_INFO("AngularVelocityToQuaternionDerivativeNode initialized.");
}

void AngularVelocityToQuaternionDerivativeNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!has_received_orientation_)
    {
        ROS_WARN_THROTTLE(5, "AngularVelocityToQuaternionDerivativeNode: Waiting for initial orientation message.");
        return;
    }

    // Extract angular velocity from IMU message
    tf2::Vector3 angular_velocity(msg->angular_velocity.x,
                                  msg->angular_velocity.y,
                                  msg->angular_velocity.z);

    // Create an angular velocity quaternion (pure quaternion with 0 scalar part)
    tf2::Quaternion omega_quat(angular_velocity.x(), angular_velocity.y(), angular_velocity.z(), 0.0);

    // Calculate quaternion derivative: dq/dt = 0.5 * q * omega_quat
    tf2::Quaternion q_derivative = latest_orientation_ * omega_quat;
    q_derivative *= 0.5;

    // Publish the quaternion derivative
    geometry_msgs::QuaternionStamped derivative_msg;
    derivative_msg.header = msg->header;
    derivative_msg.quaternion.x = q_derivative.x();
    derivative_msg.quaternion.y = q_derivative.y();
    derivative_msg.quaternion.z = q_derivative.z();
    derivative_msg.quaternion.w = q_derivative.w();

    quaternion_derivative_pub_.publish(derivative_msg);
}

void AngularVelocityToQuaternionDerivativeNode::orientationCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    tf2::fromMsg(msg->quaternion, latest_orientation_);
    latest_orientation_timestamp_ = msg->header.stamp;
    has_received_orientation_ = true;
}

} // namespace quaternion_integrator
