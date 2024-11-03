#ifndef MYLAW_H
#define MYLAW_H

#include "StateSpace.h"
#include <Eigen/Dense>
#include <fstream>

class MyLaw : public Observer::StateSpace {
public:
    Eigen::Vector3f kp_rot;
    Eigen::Vector3f kd_rot_1;
    Eigen::Vector3f kd_rot_2;
    float sat_rot;

    Eigen::Vector3f kp_trans;
    Eigen::Vector3f kd_trans_1;
    Eigen::Vector3f kd_trans_2;
    float sat_trans;

    Eigen::Vector3f omega_gains_trans;

    Eigen::Quaternionf eq;

    Eigen::Vector3f p_d;
    
    float Fu;
    Eigen::Vector3f Tauu;
    std::ofstream controlOutputFile;
    std::string controlFilePath;

    MyLaw();
    ~MyLaw();

    void CalculateControl(const Eigen::MatrixXf&, float);
    Eigen::Quaternionf ExpQuaternion(const Eigen::Quaternionf&);
    Eigen::Vector3f rotvec(const Eigen::Quaternionf&);
    Eigen::Quaternionf LogQuaternion(const Eigen::Quaternionf&);
    void saveControlDataToCSV();
};

#endif // MYLAW_H
