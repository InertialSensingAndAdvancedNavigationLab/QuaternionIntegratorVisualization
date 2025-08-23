#include "QuaternionIntegratorNode.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "quaternion_integrator_node");
    ros::NodeHandle nh;
    quaternion_integrator::QuaternionIntegratorNode node(nh);
    ros::spin();
    return 0;
}
