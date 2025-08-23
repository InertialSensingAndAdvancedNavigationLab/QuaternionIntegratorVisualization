#include <ros/ros.h>
#include "ImuCorrectorNode.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_corrector_node");
    ros::NodeHandle nh;
    quaternion_integrator::ImuCorrectorNode imu_corrector_node(nh);
    ros::spin();
    return 0;
}