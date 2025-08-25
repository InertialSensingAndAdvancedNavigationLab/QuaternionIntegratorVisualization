/**
 * @file AngularVelocityToQuaternionDerivativeNode.hpp
 * @brief 角速度到四元数微分转换节点的头文件
 *
 * @details 该文件定义了 `AngularVelocityToQuaternionDerivativeNode` 类,
 * 该类是一个ROS节点，负责将输入的角速度（`geometry_msgs::Vector3Stamped`）
 * 转换为四元数的时间微分（`geometry_msgs::Quaternion`），并将其发布。
 * 这是四元数积分法进行姿态估计的第一步。
 **/
#ifndef QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
#define QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @brief quaternion_integrator 命名空间
 * @details 包含Winyunq项目（或Euroc_ws项目）中所有与四元数积分相关的功能。
 **/
namespace quaternion_integrator {

/**
 * @class AngularVelocityToQuaternionDerivativeNode
 * @brief 将角速度转换为四元数微分的ROS节点类。
 * @details 该节点订阅一个表示角速度的 `geometry_msgs::Vector3Stamped` 消息，
 *          根据运动学方程 `dq/dt = 0.5 * q * [0, wx, wy, wz]` 计算出
 *          当前姿态（假设为单位四元数）下的四元数时间微分，
 *          并将结果作为 `geometry_msgs::Quaternion` 消息发布。
 **/
class AngularVelocityToQuaternionDerivativeNode {
public:
    /**
     * @brief       构造函数
     * @details     初始化ROS节点，设置订阅者和发布者。
     *              从ROS参数服务器读取输入和输出的话题名称。
     *
     * @param       nh                              数据类型: const ros::NodeHandle&
     * @details     ROS节点句柄的常量引用，用于与ROS系统交互。
     **/
    AngularVelocityToQuaternionDerivativeNode(const ros::NodeHandle& nh);

    /**
     * @brief 默认析构函数
     * @details 使用C++11的default关键字，让编译器生成默认的析构函数。
     **/
    ~AngularVelocityToQuaternionDerivativeNode() = default;

private:
    /**
     * @brief       角速度消息的回调函数
     * @details     当接收到新的IMU消息时，此函数被调用。
     *              它从IMU消息中提取角速度，执行从角速度到四元数微分的转换，并发布结果。
     * @warning     此实现假设当前的姿态四元数是单位四元数 [1, 0, 0, 0]，
     *              这在很多从静止开始的简单场景下是有效的。
     *              对于更复杂的场景，需要一个完整的姿态积分器来提供当前姿态。
     *
     * @param       msg                             数据类型: const sensor_msgs::Imu::ConstPtr&
     * @details     指向接收到的IMU消息的常量共享指针，其中包含角速度。
     **/
    void angularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief       姿态消息的回调函数
     * @details     当接收到新的姿态消息时，此函数被调用。
     *              它从消息中提取当前的姿态四元数并存储起来。
     * @param       msg                             数据类型: const geometry_msgs::PoseStamped::ConstPtr&
     * @details     指向接收到的姿态消息的常量共享指针。
     **/
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /// @brief ROS节点句柄：用于初始化节点、订阅和发布话题等
    ros::NodeHandle nh_;
    /// @brief 角速度订阅者：订阅输入的角速度话题
    ros::Subscriber sub_angular_velocity_;
    /// @brief 姿态订阅者：订阅输入的姿态话题
    ros::Subscriber sub_pose_;
    /// @brief 四元数微分发布者：发布计算出的四元数微分话题
    ros::Publisher pub_quaternion_derivative_;
    /// @brief 当前姿态四元数
    geometry_msgs::Quaternion current_orientation_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
