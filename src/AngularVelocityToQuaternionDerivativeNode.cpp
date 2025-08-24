#include "AngularVelocityToQuaternionDerivativeNode.hpp"
#include <sensor_msgs/Imu.h> // 包含IMU消息类型

namespace quaternion_integrator {

/// @brief 构造函数实现
AngularVelocityToQuaternionDerivativeNode::AngularVelocityToQuaternionDerivativeNode(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    /// 从参数服务器获取输入话题名称，默认为 "/imu/data_corrected"
    std::string imu_input_topic;
    nh_private.param<std::string>("imu_input_topic", imu_input_topic, "/imu/data_corrected");

    /// 从参数服务器获取输出话题名称，默认为 "/imu/quaternion_derivative"
    std::string quaternion_derivative_output_topic;
    nh_private.param<std::string>("quaternion_derivative_output_topic", quaternion_derivative_output_topic, "/imu/quaternion_derivative");

    /// 初始化订阅者，订阅IMU输入话题
    sub_angular_velocity_ = nh_.subscribe(imu_input_topic, 10, &AngularVelocityToQuaternionDerivativeNode::angularVelocityCallback, this);

    /// 初始化发布者，发布四元数微分话题
    pub_quaternion_derivative_ = nh_.advertise<geometry_msgs::Quaternion>(quaternion_derivative_output_topic, 10);

    ROS_INFO("[AngularVelocityToQuaternionDerivativeNode] Node initialized. Subscribing to %s and publishing to %s", imu_input_topic.c_str(), quaternion_derivative_output_topic.c_str());
}

/// @brief 角速度消息的回调函数实现
void AngularVelocityToQuaternionDerivativeNode::angularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    /// 创建一个geometry_msgs::Quaternion消息用于存储四元数微分
    geometry_msgs::Quaternion quaternion_derivative_msg;

    /// 根据 dq/dt = 0.5 * q * [0, wx, wy, wz] 公式计算四元数微分
    /// 警告：此节点假设当前姿态q为单位四元数 [1, 0, 0, 0]，因此 q * [0, wx, wy, wz] 简化为 [0, wx, wy, wz]
    /// 故四元数微分的w分量为0，x,y,z分量为角速度的一半
    quaternion_derivative_msg.w = 0.0;
    quaternion_derivative_msg.x = 0.5 * msg->angular_velocity.x;
    quaternion_derivative_msg.y = 0.5 * msg->angular_velocity.y;
    quaternion_derivative_msg.z = 0.5 * msg->angular_velocity.z;

    /// 发布计算出的四元数微分
    pub_quaternion_derivative_.publish(quaternion_derivative_msg);
}

} // namespace quaternion_integrator
