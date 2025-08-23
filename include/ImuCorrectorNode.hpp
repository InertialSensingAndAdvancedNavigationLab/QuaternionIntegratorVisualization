/**
 * @file ImuCorrectorNode.hpp
 * @brief IMU数据校正节点的头文件
 *
 * @details 该文件定义了 `ImuCorrectorNode` 类。
 * 这个ROS节点负责订阅原始IMU数据，应用校正算法（如偏差补偿、噪声滤波），
 * 并发布校正后的IMU数据。
 **/
#ifndef QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
#define QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace quaternion_integrator {

/**
 * @class ImuCorrectorNode
 * @brief 负责IMU数据校正的ROS节点类。
 * @details 该节点执行以下操作：
 *          1. 订阅原始IMU数据。
 *          2. 应用预定义的校正（例如，偏差补偿）。
 *          3. 发布校正后的IMU数据。
 *          4. 计算并发布IMU姿态的Roll, Pitch, Yaw，以及旋转矩阵。
 **/
class ImuCorrectorNode {
public:
    /**
     * @brief       构造函数
     * @details     初始化ROS节点，设置订阅者和发布者，并加载参数。
     *
     * @param       nh                              数据类型: const ros::NodeHandle&
     * @details     ROS节点句柄的常量引用，用于与ROS系统交互。
     **/
    ImuCorrectorNode(const ros::NodeHandle& nh);

private:
    /**
     * @brief       原始IMU数据的回调函数
     * @details     当接收到原始IMU数据时，此函数将应用校正并发布校正后的数据。
     *              同时，它会计算并发布IMU姿态的Roll, Pitch, Yaw，以及旋转矩阵。
     *
     * @param       msg                             数据类型: const sensor_msgs::Imu::ConstPtr&
     * @details     指向接收到的原始IMU消息的常量共享指针。
     **/
    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_raw_;
    ros::Publisher pub_imu_corrected_;
    ros::Publisher pub_rpy_;
    ros::Publisher pub_quaternion_stamped_;

    // Parameters
    std::string input_imu_topic_;
    std::string output_imu_topic_;
    double angular_velocity_bias_x_;
    double angular_velocity_bias_y_;
    double angular_velocity_bias_z_;
    double linear_acceleration_bias_x_;
    double linear_acceleration_bias_y_;
    double linear_acceleration_bias_z_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
