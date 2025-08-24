#include "PoseEstimatorNode.hpp"

namespace quaternion_integrator {

PoseEstimatorNode::PoseEstimatorNode(const ros::NodeHandle& nh) : nh_(nh), tf_broadcaster_() {
    ros::NodeHandle nh_private("~");
    std::string quaternion_input_topic;
    nh_private.param<std::string>("quaternion_input_topic", quaternion_input_topic, "/imu/quaternion");

    std::string imu_input_topic;
    nh_private.param<std::string>("imu_input_topic", imu_input_topic, "/imu/data_corrected");

    std::string pose_output_topic;
    nh_private.param<std::string>("pose_output_topic", pose_output_topic, "/imu/pose");

    std::string rpy_output_topic;
    nh_private.param<std::string>("rpy_output_topic", rpy_output_topic, "/imu/rpy");

    std::string marker_output_topic;
    nh_private.param<std::string>("marker_output_topic", marker_output_topic, "/visualization/ahrs_drone");

    sub_integrated_quaternion_ = nh_.subscribe(quaternion_input_topic, 10, &PoseEstimatorNode::integratedQuaternionCallback, this);
    sub_corrected_imu_ = nh_.subscribe(imu_input_topic, 10, &PoseEstimatorNode::correctedImuCallback, this);

    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_output_topic, 10, true);
    pub_rpy_ = nh_.advertise<geometry_msgs::Vector3Stamped>(rpy_output_topic, 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_output_topic, 10, true);

    ROS_INFO("[PoseEstimatorNode] Node initialized. Subscribing to %s and %s, publishing to %s, %s, and %s",
             quaternion_input_topic.c_str(), imu_input_topic.c_str(), pose_output_topic.c_str(), rpy_output_topic.c_str(), marker_output_topic.c_str());
}

void PoseEstimatorNode::integratedQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    ROS_INFO_ONCE("[PoseEstimatorNode] First quaternion message received. Processing...");

    // -- Coordinate System Correction and TF/Pose Publishing using TF2 --
    tf2::Quaternion tf_quat_original;
    tf2::fromMsg(msg->quaternion, tf_quat_original);

    // Correction from URF (Up-Right-Forward) to FLU (Forward-Left-Up)
    tf2::Quaternion tf_quat_correction(0.70710678, 0.0, 0.70710678, 0.0);
    tf2::Quaternion tf_quat_final = tf_quat_original * tf_quat_correction;
    tf_quat_final.normalize();

    geometry_msgs::Quaternion final_orientation = tf2::toMsg(tf_quat_final);

    // Publish PoseStamped
    ROS_INFO_THROTTLE(1.0, "[PoseEstimatorNode] Publishing /imu/pose...");
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.orientation = final_orientation;
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pub_pose_.publish(pose_msg);

    // Publish RPY
    ROS_INFO_THROTTLE(1.0, "[PoseEstimatorNode] Publishing /imu/rpy...");
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat_final).getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header.stamp = msg->header.stamp;
    rpy_msg.header.frame_id = "ahrs_body";
    rpy_msg.vector.x = roll;
    rpy_msg.vector.y = pitch;
    rpy_msg.vector.z = yaw;
    pub_rpy_.publish(rpy_msg);

    // Broadcast TF transform using TF2
    ROS_INFO_THROTTLE(1.0, "[PoseEstimatorNode] Broadcasting TF transform...");
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "ahrs_body";
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = final_orientation;
    tf_broadcaster_.sendTransform(transform_stamped);

    // -- Visualization Marker Publishing --
    ROS_INFO_THROTTLE(1.0, "[PoseEstimatorNode] Publishing markers...");
    visualization_msgs::Marker body_disk;
    body_disk.header.frame_id = "ahrs_body";
    body_disk.header.stamp = msg->header.stamp;
    body_disk.ns = "ahrs_drone_body";
    body_disk.id = 0;
    body_disk.type = visualization_msgs::Marker::CYLINDER;
    body_disk.action = visualization_msgs::Marker::ADD;
    body_disk.pose.orientation.w = 1.0;
    body_disk.scale.x = 0.8;
    body_disk.scale.y = 0.8;
    body_disk.scale.z = 0.02;
    body_disk.color.a = 0.7;
    body_disk.color.r = 0.8; 
    body_disk.color.g = 0.8; 
    body_disk.color.b = 0.8; // Gray
    pub_marker_.publish(body_disk);

    visualization_msgs::Marker down_cylinder;
    down_cylinder.header.frame_id = "ahrs_body";
    down_cylinder.header.stamp = msg->header.stamp;
    down_cylinder.ns = "ahrs_drone_body";
    down_cylinder.id = 1;
    down_cylinder.type = visualization_msgs::Marker::CYLINDER;
    down_cylinder.action = visualization_msgs::Marker::ADD;
    down_cylinder.pose.position.z = -0.11;
    down_cylinder.pose.orientation.w = 1.0;
    down_cylinder.scale.x = 0.05;
    down_cylinder.scale.y = 0.05;
    down_cylinder.scale.z = 0.2;
    down_cylinder.color.a = 0.9;
    down_cylinder.color.r = 0.5;
    down_cylinder.color.g = 0.5;
    down_cylinder.color.b = 0.5;
    pub_marker_.publish(down_cylinder);

    visualization_msgs::Marker forward_arrow;
    forward_arrow.header.frame_id = "ahrs_body";
    forward_arrow.header.stamp = msg->header.stamp;
    forward_arrow.ns = "ahrs_drone_body";
    forward_arrow.id = 2;
    forward_arrow.type = visualization_msgs::Marker::ARROW;
    forward_arrow.action = visualization_msgs::Marker::ADD;
    forward_arrow.pose.position.z = -0.11;
    forward_arrow.pose.orientation.w = 1.0;
    forward_arrow.scale.x = 0.4;
    forward_arrow.scale.y = 0.05;
    forward_arrow.scale.z = 0.05;
    forward_arrow.color.a = 1.0;
    forward_arrow.color.r = 1.0;
    forward_arrow.color.g = 0.0;
    forward_arrow.color.b = 0.0;
    pub_marker_.publish(forward_arrow);
    ROS_INFO_THROTTLE(1.0, "[PoseEstimatorNode] All publications in callback are done.");
}

void PoseEstimatorNode::correctedImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    latest_imu_ = *msg;
}

} // namespace quaternion_integrator
