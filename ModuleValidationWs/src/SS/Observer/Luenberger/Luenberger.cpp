#include "Luenberger.h"

namespace Observer {

Luenberger::Luenberger() {
    firstIteration_trans = true;
    firstIteration_rot = true;

    // Initialize extended matrices for translational dynamics
    A_ext_trans = Eigen::MatrixXf::Zero(12, 12);
    B_ext_trans = Eigen::MatrixXf::Zero(12, 3);
    C_ext_trans = Eigen::MatrixXf::Zero(3, 12);

    // Initialize extended matrices for rotational dynamics
    A_ext_rot = Eigen::MatrixXf::Zero(12, 12);
    B_ext_rot = Eigen::MatrixXf::Zero(12, 3);
    C_ext_rot = Eigen::MatrixXf::Zero(3, 12);

    // Initialize observer gains
    L_trans = Eigen::MatrixXf::Zero(12, 3);
    L_rot = Eigen::MatrixXf::Zero(12, 3);

    // Initialize state estimates
    x_trans = Eigen::VectorXf::Zero(12);
    x_rot = Eigen::VectorXf::Zero(12);

    InitializeEstimates();
}

Luenberger::~Luenberger() {}

void Luenberger::InitializeEstimates() {
    x_trans.setZero();
    x_rot.setZero();

    // Set extended dynamics matrices for translational and rotational dynamics
    A_ext_trans.block<6, 6>(0, 0) = A_trans;
    A_ext_trans.block<6, 3>(0, 6) = B_trans;
    A_ext_trans.block<3, 3>(6, 9) = Eigen::MatrixXf::Identity(3, 3);
    A_ext_trans.block<3, 3>(9, 6) = -Eigen::MatrixXf::Identity(3, 3);

    A_ext_rot.block<6, 6>(0, 0) = A_rot;
    A_ext_rot.block<6, 3>(0, 6) = B_rot;
    A_ext_rot.block<3, 3>(6, 9) = Eigen::MatrixXf::Identity(3, 3);
    A_ext_rot.block<3, 3>(9, 6) = -Eigen::MatrixXf::Identity(3, 3);

    B_ext_trans.block<6, 3>(0, 0) = B_trans;
    B_ext_rot.block<6, 3>(0, 0) = B_rot;

    C_ext_trans.block<3, 6>(0, 0) = C_trans; 

    C_ext_rot.block<3, 6>(0, 0) = C_rot; 

    L_trans << 
        33.0000000000006,    4.00998690134642e-13,    1.00722372192219e-12,
         8.81702467836799e-13,   33.0000000000013,   9.09013215467378e-14,
        -1.66232183786225e-12,   3.14350806617424e-13,   32.9999999999998,
        364.000000000012,    9.05813626265392e-12,    2.36372536951903e-11,
         2.13271274497135e-11,   364.000000000026,   2.50829140876186e-12,
        -3.50492607120818e-11,   6.23734589091321e-12,   363.999999999995,
        624.510000000031,    2.08162765191863e-11,    6.50881618343760e-11,
         5.71016255386048e-11,   624.510000000060,   7.91571186271282e-12,
        -9.15008181838399e-11,   1.45175199369876e-11,   624.509999999988,
        763.830000000055,    3.05005650142050e-11,    1.11563904986068e-10,
         9.69958064121538e-11,   763.830000000091,   1.55743243790425e-11,
        -1.53553592474806e-10,   2.18552153875592e-11,   763.829999999981;


    L_rot << 27.9749, 1.2599, -3.3911,
             1.7784, 31.8721, -3.1220,
            -1.7279, -0.6962, 30.1531,
            269.3539, 30.8038, -71.8517,
             43.8236, 360.5651, -73.9444,
            -36.6802, -16.7090, 322.1691,
              2.1655, 0.5958, -1.0245,
              0.7639, 3.5478, -1.1837,
             -0.9753, -0.5153, 5.7455,
              2.3080, 1.2387, -1.9486,
              1.6142, 5.1977, -2.6013,
             -1.8207, -1.1728, 7.7446;
}

void Luenberger::SetNewGainL_trans(const Eigen::MatrixXf& L_trans_new) {
    L_trans = L_trans_new;
}

void Luenberger::SetNewGainL_rot(const Eigen::MatrixXf& L_rot_new) {
    L_rot = L_rot_new;
}

Eigen::Vector3f Luenberger::EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    Eigen::VectorXf x_t(12);
    x_t << p, dp, Eigen::VectorXf::Zero(6);
    Eigen::Vector3f y = C_ext_trans * x_t;
    Eigen::VectorXf dx_hat_ext_L_trans = 
        A_ext_trans * x_trans + 
        B_ext_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + 
        0*L_trans * (y - C_ext_trans * x_trans);

    std::cout << y << std::endl<< std::endl;
    x_trans += dt * dx_hat_ext_L_trans;
    Eigen::Vector3f disturbance_estimate = x_trans.segment<3>(6);
    SaveStateEstimationCSV(x_trans, dx_hat_ext_L_trans, disturbance_estimate, "TranslationalEstimation.csv");
    return disturbance_estimate;
    return Eigen::Vector3f(0,0,0);
}


Eigen::Vector3f Luenberger::EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::VectorXf x_r(12);
    x_r << q.vec(), omega, Eigen::VectorXf::Zero(6);
    Eigen::Vector3f y = C_ext_rot * x_r;
    Eigen::VectorXf dx_hat = A_ext_rot * x_rot + B_ext_rot * u_torque + L_rot * (y - C_ext_rot * x_rot);
    x_rot += dt * dx_hat;
    Eigen::Vector3f disturbance_estimate = x_rot.segment<3>(6);
    SaveStateEstimationCSV(x_rot, dx_hat, disturbance_estimate, "RotationalEstimation.csv");
    return disturbance_estimate;
}

} // namespace Observer
