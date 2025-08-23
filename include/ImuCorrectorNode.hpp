/**
 * @file ImuCorrectorNode.hpp
 * @brief IMU数据校正节点的头文件
 *
 * @details 该文件定义了 `ImuCorrectorNode` 类。
 * 这个ROS节点负责订阅原始的IMU数据 (`sensor_msgs::Imu`),
 * 对其进行零偏（bias）和漂移（drift）校正，然后以多种格式发布处理后的数据。
 * 同时，它还提供一个ROS服务，允许外部节点动态地设置零偏和漂移参数。
 **/
#ifndef QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
#define QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "quaternion_integrator/SetBiasDrift.h"

/**
 * @brief quaternion_integrator 命名空间
 * @details 包含Winyunq项目（或Euroc_ws项目）中所有与四元数积分相关的功能。
 **/
namespace quaternion_integrator {

/**
 * @class ImuCorrectorNode
 * @brief 对原始IMU数据进行校正和滤波的ROS节点类。
 * @details 该节点的核心功能是提高IMU数据的质量。它通过以下方式实现：
 *          1. 订阅原始IMU数据。
 *          2. 减去当前的角速度和线加速度零偏。
 *          3. （可选）补偿已知的传感器漂移。
 *          4. 发布校正后的完整IMU消息。
 *          5. 单独发布校正后的角速度和线加速度，方便下游节点使用。
 *          6. 提供一个名为 `set_bias_drift` 的服务，用于在运行时更新校正参数。
 **/
class ImuCorrectorNode {
public:
    /**
     * @brief       构造函数
     * @details     初始化ROS节点，从参数服务器加载初始的零偏和漂移值，
     *              并设置所有的订阅者、发布者和服务服务器。
     *
     * @param       nh                              数据类型: const ros::NodeHandle&
     * @details     ROS节点句柄的常量引用，用于与ROS系统交互。
     **/
    ImuCorrectorNode(const ros::NodeHandle& nh);

    /**
     * @brief 默认析构函数
     * @details 使用C++11的default关键字，让编译器生成默认的析构函数。
     **/
    ~ImuCorrectorNode() = default;

private:
    /**
     * @brief       原始IMU数据的回调函数
     * @details     当接收到新的原始IMU消息时，此函数被调用。
     *              它应用当前的零偏和漂移校正，然后发布所有处理后的话题。
     *
     * @param       msg                             数据类型: const sensor_msgs::Imu::ConstPtr&
     * @details     指向接收到的IMU消息的常量共享指针。
     **/
    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief       设置零偏和漂移的服务回调函数
     * @details     当有客户端调用 `set_bias_drift` 服务时，此函数被执行。
     *              它会更新内部存储的零偏和漂移参数。
     * @note        更新后的参数将立即应用于后续接收到的所有IMU消息。
     *
     * @param       req                             数据类型: quaternion_integrator::SetBiasDrift::Request&
     * @details     服务请求对象，包含了新的零偏和漂移值。
     * @param       res                             数据类型: quaternion_integrator::SetBiasDrift::Response&
     * @details     服务响应对象，用于设置操作是否成功的标志和消息。
     *
     * @return      操作是否成功                      数据类型: bool
     * @retval      true 服务成功执行。
     * @retval      false 服务执行失败（虽然在此实现中总是返回true）。
     **/
    bool setBiasDrift(quaternion_integrator::SetBiasDrift::Request &req,
                      quaternion_integrator::SetBiasDrift::Response &res);

    /// @brief ROS节点句柄：用于初始化节点、订阅和发布话题等
    ros::NodeHandle nh_;
    /// @brief 原始IMU数据订阅者：订阅来自IMU驱动的原始数据
    ros::Subscriber sub_imu_raw_;
    /// @brief 处理后IMU数据发布者：发布经过校正的完整IMU消息
    ros::Publisher pub_imu_processed_;
    /// @brief 处理后角速度发布者：单独发布校正后的角速度向量
    ros::Publisher pub_angular_velocity_;
    /// @brief 处理后线加速度发布者：单独发布校正后的线加速度向量
    ros::Publisher pub_linear_acceleration_;
    /// @brief IMU零偏参数发布者：可以周期性地发布当前的零偏设置（可选功能）
    ros::Publisher pub_bias_params_;
    /// @brief 设置零偏和漂移的服务服务器：允许外部节点动态更新校正参数
    ros::ServiceServer srv_set_bias_drift_;

    /// @brief 角速度零偏（Bias）：一个将被从原始角速度读数中减去的向量
    geometry_msgs::Vector3 angular_velocity_bias_;
    /// @brief 线加速度零偏（Bias）：一个将被从原始线加速度读数中减去的向量
    geometry_msgs::Vector3 linear_acceleration_bias_;
    /// @brief 角速度漂移（Drift）：表示角速度随时间变化的速率（在此节点中可能未完全实现）
    geometry_msgs::Vector3 angular_velocity_drift_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
