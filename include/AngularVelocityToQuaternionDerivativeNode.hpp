#ifndef QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
#define QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>

namespace quaternion_integrator {

/**
 * @class AngularVelocityToQuaternionDerivativeNode
 * @brief 将角速度转换为四元数微分。
 * @details 该节点订阅角速度信息，计算相应的四元数时间微分，
 *          并将结果发布。这是一个核心的数学转换模块。
 */
class AngularVelocityToQuaternionDerivativeNode {
public:
    /**
     * @brief 构造函数。
     * @param nh ROS节点句柄。
     */
    AngularVelocityToQuaternionDerivativeNode(const ros::NodeHandle& nh);

    /**
     * @brief 析构函数。
     */
    ~AngularVelocityToQuaternionDerivativeNode() = default;

private:
    /**
     * @brief 角速度数据的回调函数。
     * @param msg 接收到的角速度消息。
     */
    void angularVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_angular_velocity_;
    ros::Publisher pub_quaternion_derivative_;
};

} // namespace quaternion_integrator

#endif // QUATERNION_INTEGRATOR_ANGULAR_VELOCITY_TO_QUATERNION_DERIVATIVE_NODE_HPP
