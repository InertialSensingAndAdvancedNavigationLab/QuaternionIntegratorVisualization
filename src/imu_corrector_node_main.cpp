#include "ImuCorrectorNode.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_corrector_node");
    ros::NodeHandle nh;
    quaternion_integrator::ImuCorrectorNode node(nh);
    ros::spin();
    return 0;
}
