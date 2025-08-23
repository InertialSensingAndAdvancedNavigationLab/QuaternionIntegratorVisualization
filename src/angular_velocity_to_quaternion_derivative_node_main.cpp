/**
 * @file angular_velocity_to_quaternion_derivative_node_main.cpp
 * @brief Main function to run the AngularVelocityToQuaternionDerivativeNode.
 * @author Gemini
 * @date 2025-08-23
 */
#include "quaternion_integrator/AngularVelocityToQuaternionDerivativeNode.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angular_velocity_to_quaternion_derivative_node");
    ros::NodeHandle nh;
    quaternion_integrator::AngularVelocityToQuaternionDerivativeNode node(nh);
    ros::spin();
    return 0;
}
