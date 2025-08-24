/**
 * @file PoseEstimatorNode.hpp
 * @brief 姿态估计节点的头文件
 *
 * @details 该文件定义了 `PoseEstimatorNode` 类。
 * 该ROS节点负责接收积分后的四元数和校正后的IMU数据，
 * 通过融合算法（例如互补滤波）估计设备的最终姿态和位置，
 * 并将其发布。
 **/
#ifndef QUATERNION_INTEGRATOR_POSE_ESTIMATOR_NODE_HPP
#define QUATERNION_INTEGRATOR_POSE_ESTIMATOR_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h> // 订阅积分后的四元数
#include <sensor_msgs/Imu.h> // 订阅校正后的IMU数据（用于加速度）
#include <geometry_msgs/PoseStamped.h> // 发布最终姿态和位置
#include <geometry_msgs/Vector3Stamped.h> // 发布RPY
#include <visualization_msgs/Marker.h> // 为可视化新增
#include <tf/transform_broadcaster.h> // 用于TF广播
#include <tf/transform_datatypes.h> // 用于TF数据类型转换

/**
 * @brief quaternion_integrator 命名空间
 * @details 包含Winyunq项目（或Euroc_ws项目）中所有与四元数积分和姿态估计相关的功能。
 **/
namespace quaternion_integrator {

/**
 * @class PoseEstimatorNode
 * @brief 融合四元数和IMU数据以估计最终姿态和位置的ROS节点类。
 * @details 该节点执行以下操作：
 *          1. 订阅来自 `QuaternionIntegratorNode` 的积分后的四元数。
 *          2. 订阅来自 `ImuCorrectorNode` 的校正后的IMU数据（包含加速度）。
 *          3. 融合这些数据以估计更精确的姿态和位置。
 *          4. 将估计的姿态和位置作为 `geometry_msgs::PoseStamped` 发布。
 *          5. 将姿态转换为欧拉角（RPY）并作为 `geometry_msgs::Vector3Stamped` 发布。
 *          6. 发布一个 `visualization_msgs::Marker` 消息用于在RViz中可视化。
 **/
class PoseEstimatorNode {
public:
    /**
     * @brief       构造函数
     * @details     初始化ROS节点，设置订阅者和发布者。
     *
     * @param       nh                              数据类型: const ros::NodeHandle&
     * @details     ROS节点句柄的常量引用，用于与ROS系统交互。
     **/
    PoseEstimatorNode(const ros::NodeHandle& nh);

    /**
     * @brief 默认析构函数
     * @details 使用C++11的default关键字，让编译器生成默认的析构函数。
     **/
    ~PoseEstimatorNode() = default;

private:
    /**
     * @brief       积分后四元数的回调函数
     * @details     当接收到新的积分后四元数消息时，此函数被调用。
     *              它将更新内部的姿态估计。
     *
     * @param       msg                             数据类型: const geometry_msgs::QuaternionStamped::ConstPtr&
     * @details     指向接收到的四元数消息的常量共享指针。
     **/
    void integratedQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

    /**
     * @brief       校正后IMU数据的回调函数
     * @details     当接收到新的校正后IMU消息时，此函数被调用。
     *              它将使用加速度数据来辅助姿态估计（例如，通过互补滤波）。
     *
     * @param       msg                             数据类型: const sensor_msgs::Imu::ConstPtr&
     * @details     指向接收到的IMU消息的常量共享指针。
     **/
    void correctedImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /// @brief ROS节点句柄：用于初始化节点、订阅和发布话题等
    ros::NodeHandle nh_;
    /// @brief 积分后四元数订阅者
    ros::Subscriber sub_integrated_quaternion_;
    /// @brief 校正后IMU数据订阅者
    ros::Subscriber sub_corrected_imu_;
    /// @brief 姿态发布者：发布最终的姿态和位置信息（PoseStamped）
    ros::Publisher pub_pose_;
    /// @brief RPY发布者：发布姿态的欧拉角表示
    ros::Publisher pub_rpy_;
    /// @brief Marker发布者：用于在RViz中进行可视化
    ros::Publisher pub_marker_;

    /// @brief TF广播器：用于发布坐标变换
    tf::TransformBroadcaster tf_broadcaster_;

    /// @brief 存储最新的积分后四元数
    geometry_msgs::Quaternion latest_quaternion_;
    /// @brief 存储最新的积分后四元数的时间戳
    ros::Time latest_quaternion_stamp_;
    /// @brief 存储最新的校正后IMU数据
    sensor_msgs::Imu latest_imu_;

    // TODO: 添加内部状态变量，例如用于融合的姿态、位置、速度等
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_POSE_ESTIMATOR_NODE_HPP
