#ifndef QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
#define QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "quaternion_integrator/SetBiasDrift.h"

namespace quaternion_integrator {

/**
 * @class ImuCorrectorNode
 * @brief 对原始IMU数据进行校正和滤波。
 * @details 该节点订阅原始IMU数据，应用零偏和漂移校正，
 *          并以多种形式发布处理后的数据，同时提供动态调整参数的服务。
 */
class ImuCorrectorNode {
public:
    /**
     * @brief 构造函数。
     * @param nh ROS节点句柄。
     */
    ImuCorrectorNode(const ros::NodeHandle& nh);

    /**
     * @brief 析构函数。
     */
    ~ImuCorrectorNode() = default;

private:
    /**
     * @brief 原始IMU数据的回调函数。
     * @param msg 接收到的IMU消息。
     */
    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief 设置零偏和漂移的服务回调函数。
     * @param req 服务请求。
     * @param res 服务响应。
     * @return bool 表示服务是否成功执行。
     */
    bool setBiasDrift(quaternion_integrator::SetBiasDrift::Request &req,
                      quaternion_integrator::SetBiasDrift::Response &res);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_raw_;
    ros::Publisher pub_imu_processed_;
    ros::Publisher pub_angular_velocity_;
    ros::Publisher pub_linear_acceleration_;
    ros::Publisher pub_bias_params_;
    ros::ServiceServer srv_set_bias_drift_;

    geometry_msgs::Vector3 angular_velocity_bias_;
    geometry_msgs::Vector3 linear_acceleration_bias_;
    geometry_msgs::Vector3 angular_velocity_drift_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_IMU_CORRECTOR_NODE_HPP
