#include "AngularVelocityToQuaternionDerivativeNode.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "angular_velocity_to_quaternion_derivative_node");
    ros::NodeHandle nh;
    quaternion_integrator::AngularVelocityToQuaternionDerivativeNode node(nh);
    ros::spin();
    return 0;
}
