/**
 * @file pose_estimator_node_main.cpp
 * @brief 姿态估计节点的ROS主文件
 *
 * @details 该文件包含 `PoseEstimatorNode` 的ROS主函数，
 * 负责初始化ROS节点，创建 `PoseEstimatorNode` 类的实例，
 * 并启动ROS消息循环。
 **/

#include <ros/ros.h>
#include "PoseEstimatorNode.hpp"

/**
 * @brief 主函数
 * @details ROS节点的入口点。初始化ROS，创建节点句柄，
 *          实例化 `PoseEstimatorNode`，并进入ROS消息循环。
 *
 * @param       argc                            数据类型: int
 * @details     命令行参数计数。
 * @param       argv                            数据类型: char**
 * @details     命令行参数数组。
 *
 * @return      int
 * @retval      0 正常退出。
 **/
int main(int argc, char** argv) {
    /// 初始化ROS节点
    ros::init(argc, argv, "pose_estimator_node");

    /// 创建节点句柄
    ros::NodeHandle nh;

    /// 实例化PoseEstimatorNode
    quaternion_integrator::PoseEstimatorNode pose_estimator_node(nh);

    /// 进入ROS消息循环，等待回调函数被调用
    ros::spin();

    return 0;
}
