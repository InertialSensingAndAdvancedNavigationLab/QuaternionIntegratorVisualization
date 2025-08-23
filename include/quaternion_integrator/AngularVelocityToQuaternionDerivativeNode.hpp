/**
 * @file AngularVelocityToQuaternionDerivativeNode.hpp
 * @brief Definition of the AngularVelocityToQuaternionDerivativeNode class.
 * @author Gemini
 * @date 2025-08-23
 */
#ifndef ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
#define ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace quaternion_integrator
{

class AngularVelocityToQuaternionDerivativeNode
{
public:
    /**
     * @brief Constructor for AngularVelocityToQuaternionDerivativeNode.
     * @param nh The ROS NodeHandle.
     */
    AngularVelocityToQuaternionDerivativeNode(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher quaternion_derivative_pub_;

    /**
     * @brief Callback for IMU messages.
     * @param msg The incoming IMU message.
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

} // namespace quaternion_integrator

#endif // ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
