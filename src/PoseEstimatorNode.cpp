#include "PoseEstimatorNode.hpp"
#include <tf/transform_datatypes.h> // For tf::createQuaternionMsgFromYaw and tf::getYaw
#include <tf/transform_broadcaster.h> // For broadcasting TF

namespace quaternion_integrator {

/// @brief 构造函数实现
PoseEstimatorNode::PoseEstimatorNode(const ros::NodeHandle& nh) : nh_(nh) {
    /// 从参数服务器获取输入话题名称：积分后的四元数
    std::string quaternion_input_topic;
    nh_.param<std::string>("~quaternion_input_topic", quaternion_input_topic, "/imu/quaternion");

    /// 从参数服务器获取输入话题名称：校正后的IMU数据
    std::string imu_input_topic;
    nh_.param<std::string>("~imu_input_topic", imu_input_topic, "/imu/data_corrected");

    /// 从参数服务器获取输出话题名称：最终姿态
    std::string pose_output_topic;
    nh_.param<std::string>("~pose_output_topic", pose_output_topic, "/imu/pose");

    /// 从参数服务器获取输出话题名称：RPY欧拉角
    std::string rpy_output_topic;
    nh_.param<std::string>("~rpy_output_topic", rpy_output_topic, "/imu/rpy");

    /// 初始化订阅者
    sub_integrated_quaternion_ = nh_.subscribe(quaternion_input_topic, 10, &PoseEstimatorNode::integratedQuaternionCallback, this);
    sub_corrected_imu_ = nh_.subscribe(imu_input_topic, 10, &PoseEstimatorNode::correctedImuCallback, this);

    /// 初始化发布者
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_output_topic, 10);
    pub_rpy_ = nh_.advertise<geometry_msgs::Vector3Stamped>(rpy_output_topic, 10);

    ROS_INFO("[PoseEstimatorNode] Node initialized. Subscribing to %s and %s, publishing to %s and %s",
             quaternion_input_topic.c_str(), imu_input_topic.c_str(), pose_output_topic.c_str(), rpy_output_topic.c_str());
}

/// @brief 积分后四元数的回调函数实现
void PoseEstimatorNode::integratedQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    /// 存储最新的积分四元数
    latest_quaternion_ = msg->quaternion;
    latest_quaternion_stamp_ = msg->header.stamp;

    /// TODO: 在这里可以加入更复杂的融合逻辑，例如与加速度计的互补滤波
    /// 目前，我们仅将四元数转换为姿态和RPY并发布

    /// 发布姿态 (PoseStamped)
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = "odom"; // 可以通过参数配置，或者使用世界坐标系
    pose_msg.pose.orientation = latest_quaternion_;
    pose_msg.pose.position.x = 0.0; // 暂时假设位置为原点
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pub_pose_.publish(pose_msg);

    /// 将四元数转换为RPY (Roll, Pitch, Yaw)
    tf::Quaternion tf_quat(latest_quaternion_.x, latest_quaternion_.y, latest_quaternion_.z, latest_quaternion_.w);
    tf::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header.stamp = msg->header.stamp;
    rpy_msg.header.frame_id = "base_link"; // RPY通常相对于机器人本体
    rpy_msg.vector.x = roll;
    rpy_msg.vector.y = pitch;
    rpy_msg.vector.z = yaw;
    pub_rpy_.publish(rpy_msg);

    /// 广播TF变换 (odom -> base_link)
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0)); // 暂时假设位置为原点
    transform.setRotation(tf_quat);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));
}

/// @brief 校正后IMU数据的回调函数实现
void PoseEstimatorNode::correctedImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    /// 存储最新的校正后IMU数据
    latest_imu_ = *msg;
    /// TODO: 在这里可以加入使用加速度计数据进行姿态修正的逻辑
    /// 例如，通过互补滤波或卡尔曼滤波来融合陀螺仪和加速度计数据
}

} // namespace quaternion_integrator
