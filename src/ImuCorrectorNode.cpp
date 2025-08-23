#include "ImuCorrectorNode.hpp"

namespace quaternion_integrator {

/**
 * @brief ImuCorrectorNode 构造函数实现
 * @details 初始化ROS节点，设置订阅者和发布者，并从ROS参数服务器加载配置参数。
 *          这些参数包括IMU的输入和输出话题名称，以及角速度和线性加速度的偏差值。
 *
 * @param nh ROS节点句柄的常量引用，用于与ROS系统交互。
 **/
ImuCorrectorNode::ImuCorrectorNode(const ros::NodeHandle& nh) : nh_(nh) {
    /// 从参数服务器加载IMU输入话题名称，默认为 "/imu/data_raw"
    nh_.param<std::string>("input_imu_topic", input_imu_topic_, "/imu/data_raw");
    /// 从参数服务器加载IMU输出话题名称，默认为 "/imu/data_corrected"
    nh_.param<std::string>("output_imu_topic", output_imu_topic_, "/imu/data_corrected");
    /// 从参数服务器加载角速度X轴偏差，默认为 0.0
    nh_.param<double>("angular_velocity_bias_x", angular_velocity_bias_x_, 0.0);
    /// 从参数服务器加载角速度Y轴偏差，默认为 0.0
    nh_.param<double>("angular_velocity_bias_y", angular_velocity_bias_y_, 0.0);
    /// 从参数服务器加载角速度Z轴偏差，默认为 0.0
    nh_.param<double>("angular_velocity_bias_z", angular_velocity_bias_z_, 0.0);
    /// 从参数服务器加载线性加速度X轴偏差，默认为 0.0
    nh_.param<double>("linear_acceleration_bias_x", linear_acceleration_bias_x_, 0.0);
    /// 从参数服务器加载线性加速度Y轴偏差，默认为 0.0
    nh_.param<double>("linear_acceleration_bias_y", linear_acceleration_bias_y_, 0.0);
    /// 从参数服务器加载线性加速度Z轴偏差，默认为 0.0
    nh_.param<double>("linear_acceleration_bias_z", linear_acceleration_bias_z_, 0.0);

    /// 设置原始IMU数据订阅者，订阅指定话题，并绑定回调函数
    sub_imu_raw_ = nh_.subscribe(input_imu_topic_, 10, &ImuCorrectorNode::imuRawCallback, this);
    /// 设置校正后IMU数据发布者，发布到指定话题
    pub_imu_corrected_ = nh_.advertise<sensor_msgs::Imu>(output_imu_topic_, 10);
    /// 设置RPY数据发布者
    pub_rpy_ = nh_.advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 10);
    /// 设置四元数数据发布者
    pub_quaternion_stamped_ = nh_.advertise<geometry_msgs::QuaternionStamped>("imu/quaternion", 10);

    /// 打印节点初始化信息到ROS日志
    ROS_INFO("ImuCorrectorNode initialized.");
    ROS_INFO("  Subscribing to: %s", input_imu_topic_.c_str());
    ROS_INFO("  Publishing to: %s", output_imu_topic_.c_str());
    ROS_INFO("  Publishing RPY to: imu/rpy");
    ROS_INFO("  Publishing Quaternion to: imu/quaternion");
}

/**
 * @brief 原始IMU数据的回调函数实现
 * @details 当接收到原始IMU数据消息时，此函数会创建一个消息副本，
 *          然后对角速度和线性加速度应用预设的偏差校正。校正后的数据随后被发布。
 *          同时，它会计算并发布IMU姿态的Roll, Pitch, Yaw，以及旋转矩阵。
 *
 * @param msg 指向接收到的原始IMU消息的常量共享指针。
 **/
void ImuCorrectorNode::imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    /// 创建原始IMU消息的副本，用于存储校正后的数据
    sensor_msgs::Imu corrected_imu_msg = *msg; 

    /// 对角速度的X、Y、Z分量应用偏差校正
    corrected_imu_msg.angular_velocity.x -= angular_velocity_bias_x_;
    corrected_imu_msg.angular_velocity.y -= angular_velocity_bias_y_;
    corrected_imu_msg.angular_velocity.z -= angular_velocity_bias_z_;

    /// 对线性加速度的X、Y、Z分量应用偏差校正
    corrected_imu_msg.linear_acceleration.x -= linear_acceleration_bias_x_;
    corrected_imu_msg.linear_acceleration.y -= linear_acceleration_bias_y_;
    corrected_imu_msg.linear_acceleration.z -= linear_acceleration_bias_z_;

    // TODO: Add more sophisticated correction/filtering here if needed

    /// 发布校正后的IMU数据
    pub_imu_corrected_.publish(corrected_imu_msg);

    // --- Visualization Data Calculation and Publishing ---

    /// 获取IMU消息中的四元数
    tf2::Quaternion q(corrected_imu_msg.orientation.x,
                      corrected_imu_msg.orientation.y,
                      corrected_imu_msg.orientation.z,
                      corrected_imu_msg.orientation.w);

    /// 将四元数转换为RPY
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    /// 创建并发布RPY消息
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = corrected_imu_msg.header;
    rpy_msg.vector.x = roll;
    rpy_msg.vector.y = pitch;
    rpy_msg.vector.z = yaw;
    pub_rpy_.publish(rpy_msg);

    /// 创建并发布QuaternionStamped消息
    geometry_msgs::QuaternionStamped quat_stamped_msg;
    quat_stamped_msg.header = corrected_imu_msg.header;
    quat_stamped_msg.quaternion = corrected_imu_msg.orientation;
    pub_quaternion_stamped_.publish(quat_stamped_msg);

    /// 打印四元数、RPY和旋转矩阵到控制台
    ROS_INFO_STREAM("--- IMU Data ---");
    ROS_INFO_STREAM("Quaternion: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]");
    ROS_INFO_STREAM("RPY (rad): [" << roll << ", " << pitch << ", " << yaw << "]");
    ROS_INFO_STREAM("RPY (deg): [" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI << ", " << yaw * 180.0 / M_PI << "]");

    /// 打印旋转矩阵
    ROS_INFO_STREAM("Rotation Matrix:");
    ROS_INFO_STREAM("  " << m[0][0] << "  " << m[0][1] << "  " << m[0][2]);
    ROS_INFO_STREAM("  " << m[1][0] << "  " << m[1][1] << "  " << m[1][2]);
    ROS_INFO_STREAM("  " << m[2][0] << "  " << m[2][1] << "  " << m[2][2]);
}

} // namespace quaternion_integrator