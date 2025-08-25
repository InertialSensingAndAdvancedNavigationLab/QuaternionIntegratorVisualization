#include "AngularVelocityToQuaternionDerivativeNode.hpp"
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/QuaternionStamped.h> // Added for QuaternionStamped

namespace quaternion_integrator {

AngularVelocityToQuaternionDerivativeNode::AngularVelocityToQuaternionDerivativeNode(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    std::string imu_input_topic;
    nh_private.param<std::string>("imu_input_topic", imu_input_topic, "/imu/data_corrected");

    std::string pose_input_topic;
    nh_private.param<std::string>("pose_input_topic", pose_input_topic, "/imu/pose");

    std::string quaternion_derivative_output_topic;
    nh_private.param<std::string>("quaternion_derivative_output_topic", quaternion_derivative_output_topic, "/imu/quaternion_derivative");

    sub_angular_velocity_ = nh_.subscribe(imu_input_topic, 10, &AngularVelocityToQuaternionDerivativeNode::angularVelocityCallback, this);
    sub_pose_ = nh_.subscribe(pose_input_topic, 10, &AngularVelocityToQuaternionDerivativeNode::poseCallback, this);

    pub_quaternion_derivative_ = nh_.advertise<geometry_msgs::QuaternionStamped>(quaternion_derivative_output_topic, 10); // Changed to QuaternionStamped

    current_orientation_.x = 0.0;
    current_orientation_.y = 0.0;
    current_orientation_.z = 0.0;
    current_orientation_.w = 1.0;

    ROS_INFO("[AngularVelocityToQuaternionDerivativeNode] Node initialized. Subscribing to %s and %s, and publishing to %s", imu_input_topic.c_str(), pose_input_topic.c_str(), quaternion_derivative_output_topic.c_str());
}

void AngularVelocityToQuaternionDerivativeNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_orientation_ = msg->pose.orientation;
}

void AngularVelocityToQuaternionDerivativeNode::angularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion omega_q(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z,
                            0.0);

    tf2::Quaternion current_q;
    tf2::fromMsg(current_orientation_, current_q);

    tf2::Quaternion q_dot = current_q * omega_q;
    q_dot *= 0.5;

    geometry_msgs::QuaternionStamped quaternion_derivative_msg; // Changed to QuaternionStamped
    quaternion_derivative_msg.header.stamp = msg->header.stamp; // Add timestamp from IMU
    quaternion_derivative_msg.header.frame_id = "imu_link"; // Add frame_id
    quaternion_derivative_msg.quaternion = tf2::toMsg(q_dot);

    pub_quaternion_derivative_.publish(quaternion_derivative_msg);
}

} // namespace quaternion_integrator
