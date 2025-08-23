#ifndef QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP
#define QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "IMUAlgorithmVisualization/SetAttitude.h"

namespace quaternion_integrator {

/**
 * @class QuaternionIntegratorNode
 * @brief 通过积分角速度计算姿态。
 * @details 该节点订阅处理后的IMU数据，执行四元数积分以估计姿态，
 *          并以纯姿态和完整位姿两种形式发布结果。同时提供设置初始姿态的服务。
 */
class QuaternionIntegratorNode {
public:
    /**
     * @brief 构造函数。
     * @param nh ROS节点句柄。
     */
    QuaternionIntegratorNode(const ros::NodeHandle& nh);

    /**
     * @brief 析构函数。
     */
    ~QuaternionIntegratorNode() = default;

private:
    /**
     * @brief 处理后IMU数据的回调函数。
     * @param msg 接收到的IMU消息。
     */
    void imuProcessedCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief 设置初始姿态的服务回调函数。
     * @param req 服务请求，包含目标姿态。
     * @param res 服务响应。
     * @return bool 表示服务是否成功执行。
     */
    bool setAttitude(IMUAlgorithmVisualization::SetAttitude::Request &req,
                     IMUAlgorithmVisualization::SetAttitude::Response &res);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_processed_;
    ros::Publisher pub_orientation_;
    ros::Publisher pub_pose_;
    ros::ServiceServer srv_set_attitude_;

    geometry_msgs::Quaternion orientation_;
    geometry_msgs::Point position_;
    ros::Time last_update_time_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_QUATERNION_INTEGRATOR_NODE_HPP
