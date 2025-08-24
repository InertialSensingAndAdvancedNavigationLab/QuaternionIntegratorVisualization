#include "QuaternionIntegratorNode.hpp"
#include <geometry_msgs/Quaternion.h> // 用于订阅四元数微分

namespace quaternion_integrator {

/// @brief 构造函数实现
QuaternionIntegratorNode::QuaternionIntegratorNode(const ros::NodeHandle& nh) : nh_(nh) {
    /// 从参数服务器获取输入话题名称，默认为 "/imu/quaternion_derivative"
    std::string quaternion_derivative_input_topic;
    nh_.param<std::string>("~quaternion_derivative_input_topic", quaternion_derivative_input_topic, "/imu/quaternion_derivative");

    /// 从参数服务器获取输出话题名称，默认为 "/imu/quaternion"
    std::string quaternion_output_topic;
    nh_.param<std::string>("~quaternion_output_topic", quaternion_output_topic, "/imu/quaternion");

    /// 初始化订阅者，订阅四元数微分话题
    sub_quaternion_derivative_ = nh_.subscribe(quaternion_derivative_input_topic, 10, &QuaternionIntegratorNode::imuProcessedCallback, this);

    /// 初始化发布者，发布积分后的四元数话题
    pub_orientation_ = nh_.advertise<geometry_msgs::QuaternionStamped>(quaternion_output_topic, 10);

    /// 初始化设置姿态的服务服务器
    srv_set_attitude_ = nh_.advertiseService("set_attitude", &QuaternionIntegratorNode::setAttitude, this);

    /// 初始化当前姿态为单位四元数
    orientation_.w = 1.0;
    orientation_.x = 0.0;
    orientation_.y = 0.0;
    orientation_.z = 0.0;

    /// 初始化上次更新时间
    last_update_time_ = ros::Time::now();

    ROS_INFO("[QuaternionIntegratorNode] Node initialized. Subscribing to %s and publishing to %s", quaternion_derivative_input_topic.c_str(), quaternion_output_topic.c_str());
}

/// @brief 四元数微分数据的回调函数实现
void QuaternionIntegratorNode::imuProcessedCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    // TODO: 实现四元数积分逻辑
}

/// @brief 设置初始姿态的服务回调函数实现
bool QuaternionIntegratorNode::setAttitude(quaternion_integrator::SetAttitude::Request &req,
                                         quaternion_integrator::SetAttitude::Response &res) {
    // TODO: 实现设置初始姿态的服务逻辑
    return true;
}

} // namespace quaternion_integrator
