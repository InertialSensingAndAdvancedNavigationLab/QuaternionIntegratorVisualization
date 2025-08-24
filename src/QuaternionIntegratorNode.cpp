#include "QuaternionIntegratorNode.hpp"
#include <geometry_msgs/Quaternion.h> // 用于订阅四元数微分
#include <cmath> // For std::sqrt

namespace quaternion_integrator {

/// @brief 构造函数实现
QuaternionIntegratorNode::QuaternionIntegratorNode(const ros::NodeHandle& nh) : nh_(nh) {
    ros::NodeHandle nh_private("~");
    /// 从参数服务器获取输入话题名称，默认为 "/imu/quaternion_derivative"
    std::string quaternion_derivative_input_topic;
    nh_private.param<std::string>("quaternion_derivative_input_topic", quaternion_derivative_input_topic, "/imu/quaternion_derivative");

    /// 从参数服务器获取输出话题名称，默认为 "/imu/quaternion"
    std::string quaternion_output_topic;
    nh_private.param<std::string>("quaternion_output_topic", quaternion_output_topic, "/imu/quaternion");

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
    /// 获取当前时间
    ros::Time current_time = ros::Time::now();

    /// 计算时间差 dt
    if (last_update_time_.isZero()) { // 处理第一个消息
        last_update_time_ = current_time;
        return; // 第一个消息不进行积分
    }
    double dt = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;

    if (dt <= 0) { // 避免dt为零或负数
        ROS_WARN_THROTTLE(1.0, "[QuaternionIntegratorNode] dt is zero or negative. Skipping integration.");
        return;
    }

    /// 获取四元数微分 (dq/dt)
    geometry_msgs::Quaternion q_dot_msg = *msg;

    /// 执行四元数积分: q_new = q_old + dq/dt * dt
    /// 注意：这里是简单的欧拉积分，对于四元数积分，更精确的方法是使用指数映射
    /// 但根据AngularVelocityToQuaternionDerivativeNode的输出，这里直接进行加法
    orientation_.w += q_dot_msg.w * dt;
    orientation_.x += q_dot_msg.x * dt;
    orientation_.y += q_dot_msg.y * dt;
    orientation_.z += q_dot_msg.z * dt;

    /// 归一化四元数以保持单位长度
    double norm = std::sqrt(orientation_.w * orientation_.w +
                            orientation_.x * orientation_.x +
                            orientation_.y * orientation_.y +
                            orientation_.z * orientation_.z);
    if (norm > 0) {
        orientation_.w /= norm;
        orientation_.x /= norm;
        orientation_.y /= norm;
        orientation_.z /= norm;
    } else {
        ROS_WARN_THROTTLE(1.0, "[QuaternionIntegratorNode] Quaternion norm is zero. Resetting to identity.");
        orientation_.w = 1.0;
        orientation_.x = 0.0;
        orientation_.y = 0.0;
        orientation_.z = 0.0;
    }

    /// 发布积分后的四元数
    geometry_msgs::QuaternionStamped integrated_quaternion_msg;
    integrated_quaternion_msg.header.stamp = current_time;
    integrated_quaternion_msg.header.frame_id = "base_link"; // 可以通过参数配置
    integrated_quaternion_msg.quaternion = orientation_;
    pub_orientation_.publish(integrated_quaternion_msg);
}

/// @brief 设置初始姿态的服务回调函数实现
bool QuaternionIntegratorNode::setAttitude(quaternion_integrator::SetAttitude::Request &req,
                                         quaternion_integrator::SetAttitude::Response &res) {
    /// 设置当前姿态为请求中的姿态
    orientation_ = req.attitude;
    ROS_INFO("[QuaternionIntegratorNode] Initial attitude set to w:%.2f, x:%.2f, y:%.2f, z:%.2f",
             orientation_.w, orientation_.x, orientation_.y, orientation_.z);
    res.success = true; // 标记服务成功
    return true;
}

} // namespace quaternion_integrator
