#include "Control/Drone.h"
#include <iostream>


int main() {
    float mass = 0.405f;
    Eigen::Matrix3f J;
    J <<    2098e-6, 63.577538e-6, -2.002648e-6,
            63.577538e-6, 2102e-6, 0.286186e-6,
            -2.002648e-6, 0.286186e-6, 4068e-6;
    float dt = 0.01f;
    int total_iterations = 4000;

    Drone drone(mass, J, dt);
    drone.kp_trans = Eigen::Vector3f(4,4,4);
    drone.kd_trans_1 = Eigen::Vector3f(1,1,1);
    drone.kd_trans_2 = Eigen::Vector3f(2,2,2);
    
    drone.kp_rot = Eigen::Vector3f(20,20,20);
    drone.kd_rot_1 = Eigen::Vector3f(2,2,2);
    drone.kd_rot_2 = Eigen::Vector3f(6,6,6);

    drone.sat_trans = 10;
    drone.sat_rot = 10;

    Eigen::Vector3f initial_position(0, 0, 0);
    Eigen::Quaternionf initial_orientation(1, 0, 0, 0);
    Eigen::Vector3f initial_velocity(0, 0, 0);
    Eigen::Vector3f initial_angular_velocity(0, 0, 0);
    drone.setInitialConditions(initial_position, initial_orientation, initial_velocity, initial_angular_velocity);

    Eigen::Vector3f target_position(1, 1, 4);
    drone.setTargetPosition(target_position);

    for (int i = 0; i < total_iterations; ++i) {
        if (i >= 2000 && i < 2500)
        {
            std::cout << "Aqui andamos" << std::endl;
            drone.transDistrubance = Eigen::Vector3f(-1,1, -2);
        }
        drone.applyControl();  
        drone.updateState();  
    }

    return 0;
}
