/**
 * @file QuaternionIntegratorNode.hpp
 * @brief 四元数积分姿态估计节点的头文件
 *
 * @details 该文件定义了 `QuaternionIntegratorNode` 类。
 * 这个ROS节点是姿态估计的核心，它订阅经过校正的IMU数据，
 * 通过对角速度进行时间积分来计算设备的三维姿态（表示为四元数）。
 * 它还提供一个服务来重置或设置初始姿态。
 **/
#ifndef QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP
#define QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h> // 用于订阅四元数微分
#include <quaternion_integrator/SetAttitude.h>

/**
 * @brief quaternion_integrator 命名空间
 * @details 包含Winyunq项目（或Euroc_ws项目）中所有与四元数积分相关的功能。
 **/
namespace quaternion_integrator {

/**
 * @class QuaternionIntegratorNode
 * @brief 通过积分角速度来计算并发布姿态的ROS节点类。
 * @details 该节点执行以下操作：
 *          1. 订阅来自 `AngularVelocityToQuaternionDerivativeNode` 的四元数微分。
 *          2. 在每次接收到数据时，使用积分法更新内部的姿态四元数。
 *          3. 将更新后的姿态作为 `geometry_msgs::QuaternionStamped` 发布。
 *          4. 提供一个 `set_attitude` 服务，允许外部节点设置或重置积分器的初始姿态。
 **/
class QuaternionIntegratorNode {
public:
    /**
     * @brief       构造函数
     * @details     初始化ROS节点，设置初始姿态为单位四元数，
     *              并设置订阅者、发布者和服务服务器。
     *
     * @param       nh                              数据类型: const ros::NodeHandle&
     * @details     ROS节点句柄的常量引用，用于与ROS系统交互。
     **/
    QuaternionIntegratorNode(const ros::NodeHandle& nh);

    /**
     * @brief 默认析构函数
     * @details 使用C++11的default关键字，让编译器生成默认的析构函数。
     **/
    ~QuaternionIntegratorNode() = default;

private:
    /**
     * @brief       四元数微分数据的回调函数
     * @details     这是节点的核心处理函数。当接收到四元数微分数据时，
     *              它计算自上次更新以来的时间差（dt），然后使用四元数微分
     *              更新姿态四元数。最后，发布新的姿态。
     *
     * @param       msg                             数据类型: const geometry_msgs::QuaternionStamped::ConstPtr&
     * @details     指向接收到的四元数微分消息的常量共享指针。
     **/
    void imuProcessedCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

    /**
     * @brief       设置初始姿态的服务回调函数
     * @details     允许外部请求重置当前姿态。这对于校准或在特定
     *              起始条件下开始积分非常有用。
     *
     * @param       req                             数据类型: IMUAlgorithmVisualization::SetAttitude::Request&
     * @details     服务请求，包含一个新的四元数作为目标姿态。
     * @param       res                             数据类型: IMUAlgorithmVisualization::SetAttitude::Response&
     * @details     服务响应，报告操作是否成功。
     *
     * @return      操作是否成功                      数据类型: bool
     * @retval      true 服务成功执行，内部姿态已更新。
     * @retval      false 服务执行失败。
     **/
    bool setAttitude(quaternion_integrator::SetAttitude::Request &req,
                     quaternion_integrator::SetAttitude::Response &res);

    /// @brief ROS节点句柄：用于初始化节点、订阅和发布话题等
    ros::NodeHandle nh_;
    /// @brief 四元数微分订阅者：订阅来自AngularVelocityToQuaternionDerivativeNode的四元数微分
    ros::Subscriber sub_quaternion_derivative_;
    /// @brief 姿态发布者：发布积分后的纯姿态信息（QuaternionStamped）
    ros::Publisher pub_orientation_;
    /// @brief 设置姿态的服务服务器：允许外部节点重置积分器的姿态
    ros::ServiceServer srv_set_attitude_;

    /// @brief 当前的姿态四元数：存储和更新节点估计的姿态
    geometry_msgs::Quaternion orientation_;
    /// @brief 上次更新时间：用于计算积分时间步长 (dt)
    ros::Time last_update_time_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP
