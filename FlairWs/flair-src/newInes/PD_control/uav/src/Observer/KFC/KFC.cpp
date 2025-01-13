// Kalman.cpp
#include "KFC.h"
#include "iostream"

namespace Observer{

KFC::KFC() 
{
    dt = 0.01;
    Fk = Eigen::MatrixXf::Identity(6, 6) + (A_trans * dt) + 0.5 * ((A_trans * A_trans * dt * dt));

    Bk = dt * B_trans;
    Hk = Eigen::MatrixXf::Identity(6, 6);

    Qk = Eigen::MatrixXf::Identity(6, 6) * 1.5;
    Rk = Eigen::MatrixXf::Identity(6, 6) * 25;

    Pk = Eigen::MatrixXf::Identity(6, 6) * 60;
    Xk = Eigen::VectorXf::Zero(6);
    Xk.head<3>() = p;
    Xk.tail<3>() = dp;

    firstUpdate = true;

    initialize();
}

KFC::~KFC() {}

void KFC::resetKFC() {
    firstUpdate = true;
}

void KFC::KFC_estimate(const Eigen::Vector3f &p, const Eigen::Vector3f &dp, const Eigen::Vector3f &u_thrust)
{
    Eigen::VectorXf X(6);
    X.head<3>() = p;
    X.tail<3>() = dp;

    if (p.isZero() & dp.isZero()) {
        Qk = Eigen::MatrixXf::Identity(6, 6) * 150;
        Rk = Eigen::MatrixXf::Identity(6, 6) * 1e10;
        X = Xk;
        //std::cout << "MAMA ESCUCHO BORROSO!\nMissing data: using model prediction" << std::endl;
    } else {
        Qk = Eigen::MatrixXf::Identity(6, 6) * 1.5;
        Rk = Eigen::MatrixXf::Identity(6, 6) * 35;
        //std::cout << "ia sirbo\n" << std::endl;
    }

    if (firstUpdate && dp.isZero() && u_thrust.isZero()) {
        Xk = Fk * Xk;
        firstUpdate = false;
    } else {
        Xk = Fk * Xk + Bk * (u_thrust - Eigen::Vector3f(0, 0, g * mass));
    }
    Pk = Fk * Pk * Fk.transpose() + Qk;

    Zk = Hk * X;
    Eigen::VectorXf Zest = Hk * Xk;
    Eigen::VectorXf Yk = Zk - Zest;

    Sk = Hk * Pk * Hk.transpose() + Rk;
    Kk = Pk * Hk.transpose() * Sk.inverse();

    Xk = Xk + Kk * Yk;
    Pk = (Eigen::MatrixXf::Identity(6, 6) - Kk * Hk) * Pk;
}

void KFC::getState(Eigen::Vector3f &p, Eigen::Vector3f &dp) const
{
    p = Xk.head<3>();
    dp = Xk.tail<3>();
}
} // namespace Observer