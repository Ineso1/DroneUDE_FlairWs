#include "MyLaw.h"
#include <iostream>
#include <fstream>

MyLaw::MyLaw() {
    eq = Eigen::Quaternionf(1,0,0,0);
    controlFilePath = CONTROL_INPUT_FILE_PATH_CSV;
    controlOutputFile.open(controlFilePath, std::ios::trunc);
    if (controlOutputFile.is_open()) {
        controlOutputFile << "Tauu_x,Tauu_y,Tauu_z,Fu\n";
    }
    controlOutputFile.close();
}

MyLaw::~MyLaw() {
    if (controlOutputFile.is_open()) {
        controlOutputFile.close();
    }
}

void MyLaw::CalculateControl(const Eigen::MatrixXf& stateM, float dt) {

    // Translational and Rotational state extraction
    Eigen::Quaternionf q(stateM(0, 0), stateM(1, 0), stateM(2, 0), stateM(3, 0));
    Eigen::Quaternionf dq(stateM(4, 0), stateM(5, 0), stateM(6, 0), stateM(7, 0));
    Eigen::Vector3f w(stateM(8, 0), stateM(9, 0), stateM(10, 0));
    Eigen::Vector3f p(stateM(11, 0), stateM(12, 0), stateM(13, 0));
    Eigen::Vector3f dp(stateM(14, 0), stateM(15, 0), stateM(16, 0));
    Eigen::Vector3f ddp(stateM(17, 0), stateM(18, 0), stateM(19, 0));

    Eigen::Vector3f w_estimation_trans = EstimateDisturbance_trans(p, dp, dt);
    Eigen::Vector3f w_estimation_rot = EstimateDisturbance_rot(q, omega, dt);

    #if OBSERVER_TYPE == UDE_OBSERVER
        w_estimation_trans = Eigen::Vector3f(0.7 * w_estimation_trans.x(), 0.7*w_estimation_trans.y(), w_estimation_trans.z());
    #endif

    #if OBSERVER_TYPE == LUENBERGER_OBSERVER
        w_estimation_trans = Eigen::Vector3f(0.7 * w_estimation_trans.x(), 0.7 * w_estimation_trans.y(), w_estimation_trans.z());
    #endif

    #if OBSERVER_TYPE == SLIDINGMODE_OBSERVER
        w_estimation_trans = Eigen::Vector3f(0.7 * w_estimation_trans.x(), 0.7 * w_estimation_trans.y(), w_estimation_trans.z());
    #endif

    #if OBSERVER_TYPE == SUPERTWIST_OBSERVER
        w_estimation_trans = Eigen::Vector3f(0.7 * w_estimation_trans.x(), 0.7 * w_estimation_trans.y(), w_estimation_trans.z());
    #endif
    
    // Errors
    Eigen::Vector3f ep = p_d - p;

    // Translational Control
    u_thrust = (kp_trans.array() * ep.array()) - (kd_trans_1.array() * dp.array());
    
    if (u_thrust.norm() != 0) {
        u_thrust = sat_trans * tanh(u_thrust.norm() / sat_trans) * u_thrust.normalized();
    }
    u_thrust += Eigen::Vector3f(0, 0, g * mass) - (kd_trans_2.array() * dp.array()).matrix() - 0*w_estimation_trans;
    Fu = u_thrust.norm();

    // Rotation Control
    Eigen::Vector3f uz_uvec = u_thrust.normalized();
    float dot_product = Eigen::Vector3f::UnitZ().dot(uz_uvec);
    Eigen::Vector3f cross_product = Eigen::Vector3f::UnitZ().cross(uz_uvec);
    Eigen::Quaternionf q_temp(dot_product, cross_product.x(), cross_product.y(), cross_product.z());
    Eigen::Quaternionf q_d = ExpQuaternion(Eigen::Quaternionf(0.5f * LogQuaternion(q_temp).coeffs()));    
    q_d.normalize();
    Eigen::Quaternionf eq_prev = eq;
    eq = q * q_d.conjugate();    
    Eigen::Vector3f eomega = rotvec(eq * eq_prev.conjugate()) / dt;
    u_torque = -kp_rot.array() * rotvec(eq).array() - kd_rot_1.array() * eomega.array();
    if (u_torque.norm() != 0) {
        u_torque = sat_rot * tanh(u_torque.norm() / sat_rot) * u_torque.normalized().array() - kd_rot_2.array() * eomega.array();
    }
    Tauu = J * u_torque + 0*w_estimation_rot;
    saveControlDataToCSV();
}

Eigen::Quaternionf MyLaw::ExpQuaternion(const Eigen::Quaternionf& q) {
    float w = q.w();
    Eigen::Vector3f v(q.x(), q.y(), q.z());
    float v_norm = v.norm();
    Eigen::Quaternionf exp_q;
    exp_q.w() = std::exp(w) * std::cos(v_norm); 
    if (v_norm > 1e-6) {
        Eigen::Vector3f exp_v = std::exp(w) * (std::sin(v_norm) / v_norm) * v;
        exp_q.vec() = exp_v; 
    } else {
        exp_q.vec() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return exp_q.normalized();
}

Eigen::Quaternionf MyLaw::LogQuaternion(const Eigen::Quaternionf& q) {
    Eigen::Quaternionf normalized_q = q.normalized();
    float w = normalized_q.w();
    Eigen::Vector3f v(normalized_q.x(), normalized_q.y(), normalized_q.z()); // Vector part
    float v_norm = v.norm();
    float theta = std::acos(w);
    Eigen::Quaternionf log_q;
    log_q.w() = 0.0f;
    if (v_norm > 1e-6) {
        Eigen::Vector3f log_v = (theta / v_norm) * v;
        log_q.vec() = log_v;
    } else {
        log_q.vec() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return log_q;
}

Eigen::Vector3f MyLaw::rotvec(const Eigen::Quaternionf &quat) {
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    double norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1e-6) {
        return Eigen::Vector3f::Zero();
    }
    double theta = 2 * std::atan2(norm, w);
    return theta * Eigen::Vector3f(x / norm, y / norm, z / norm);
}

void MyLaw::saveControlDataToCSV() {
    controlOutputFile.open(controlFilePath, std::ios::app);
    if (controlOutputFile.is_open()) {
        controlOutputFile << Tauu.x() << "," << Tauu.y() << "," << Tauu.z() << "," << Fu << "\n";
    } else {
        std::cerr << "Failed to open control_data.csv for writing.\n";
    }
    controlOutputFile.close();
}