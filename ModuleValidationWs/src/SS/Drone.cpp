#include "Drone.h"
#include <iostream>
#include <filesystem>
#include <iomanip>
#include <fstream>

Drone::Drone(float mass, const Eigen::Matrix3f& J_inertia, float timestep)
    : dt(timestep), iterations(0) {
    this->mass = mass;
    this->J = J_inertia;
    Fu = 0;
    Tauu = Eigen::Vector3f::Zero();
    p = Eigen::Vector3f::Zero();
    dp = Eigen::Vector3f::Zero();
    ddp = Eigen::Vector3f::Zero();
    q = Eigen::Quaternionf::Identity();
    omega = Eigen::Vector3f::Zero();
    domega = Eigen::Vector3f::Zero();
    u_thrust = Eigen::Vector3f::Zero();
    u_torque = Eigen::Vector3f::Zero();
    transFilePath = std::string(SOURCE_DIR) + "/src/SS/SimData/RealStateSpace_trans.csv";
    rotFilePath = std::string(SOURCE_DIR) + "/src/SS/SimData/RealStateSpace_rot.csv";
    transOutputFile.open(transFilePath, std::ios::trunc);
    rotOutputFile.open(rotFilePath, std::ios::trunc);
    transOutputFile << "p_x,p_y,p_z,dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,dt\n";
    rotOutputFile   << "domega_x,domega_y,domega_z,omega_x,omega_y,omega_z,"
                    << "dq_w,dq_x,dq_y,dq_z,q_w,q_x,q_y,q_z,dt\n";
}

Drone::~Drone() {
    if (transOutputFile.is_open()) {
        transOutputFile.close();
    }
    if (rotOutputFile.is_open()) {
        rotOutputFile.close();
    }
}

void Drone::setInitialConditions(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation,
                                 const Eigen::Vector3f& velocity, const Eigen::Vector3f& angular_velocity) {
    p = position;
    q = orientation;
    dp = velocity;
    omega = angular_velocity;
}

void Drone::setTargetPosition(const Eigen::Vector3f& target_position) {
    p_d = target_position;
}

void Drone::updateState() {
    Eigen::Quaternionf body_force_quat(0, 0, 0, Fu / mass);
    Eigen::Quaternionf rotated_force = q * body_force_quat * q.conjugate();
    ddp = Eigen::Vector3f(rotated_force.x(), rotated_force.y(), rotated_force.z());
    dp += (Eigen::Vector3f(0, 0, -g) + ddp) * dt;
    domega = J.inverse() * (u_torque - omega.cross(J * omega));
    omega += domega * dt;
    Eigen::Quaternionf omega_quat(0, 0.5f * omega.x(), 0.5f * omega.y(), 0.5f * omega.z());
    dq = omega_quat * q;
    q.coeffs() += (dq.coeffs() * dt);
    q.normalize();
    p += dp * dt + 0.5f * ddp * dt * dt;
    iterations++;
    saveStateDataToCSV();
}

void Drone::applyControl() {
    Eigen::MatrixXf stateM = getStateMatrix();
    CalculateControl(stateM, dt);
    Fu_inertial = Fu;
    u_torque = Tauu;
}

Eigen::MatrixXf Drone::getStateMatrix() const {
    Eigen::MatrixXf stateM(20, 1);

    stateM(0, 0) = q.w();
    stateM(1, 0) = q.x();
    stateM(2, 0) = q.y();
    stateM(3, 0) = q.z();
    stateM(4, 0) = dq.w();
    stateM(5, 0) = dq.x();
    stateM(6, 0) = dq.y();
    stateM(7, 0) = dq.z();
    stateM(8, 0) = omega.x();
    stateM(9, 0) = omega.y();
    stateM(10, 0) = omega.z();
    stateM(11, 0) = p.x();
    stateM(12, 0) = p.y();
    stateM(13, 0) = p.z();
    stateM(14, 0) = dp.x();
    stateM(15, 0) = dp.y();
    stateM(16, 0) = dp.z();
    stateM(17, 0) = ddp.x();
    stateM(18, 0) = ddp.y();
    stateM(19, 0) = ddp.z();
    return stateM;
}

void Drone::saveStateDataToCSV() {
    if (transOutputFile.is_open()) {
        transOutputFile << std::fixed << std::setprecision(6)
                        << p.x() << "," << p.y() << "," << p.z() << ","
                        << dp.x() << "," << dp.y() << "," << dp.z() << ","
                        << ddp.x() << "," << ddp.y() << "," << ddp.z() << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_trans.csv\n";
    }
    if (rotOutputFile.is_open()) {
        rotOutputFile << domega.x() << "," << domega.y() << "," << domega.z() << ","
                      << omega.x() << "," << omega.y() << "," << omega.z() << ","
                      << dq.w() << "," << dq.x() << "," << dq.y() << "," << dq.z() << ","
                      << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_rot.csv\n";
    }
}