#ifndef DRONE_H
#define DRONE_H

#include "MyLaw.h"
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>


class Drone : public MyLaw {
public:
    float dt;                  
    int iterations;     
    std::string transFilePath;     
    std::string rotFilePath;     
    std::ofstream transOutputFile;
    std::ofstream rotOutputFile;
    Drone(float mass, const Eigen::Matrix3f& J, float dt);
    ~Drone();
    void setInitialConditions(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation, const Eigen::Vector3f& velocity, const Eigen::Vector3f& angular_velocity);
    void setTargetPosition(const Eigen::Vector3f& target_position);
    void updateState();         
    void applyControl(); 
    Eigen::MatrixXf getStateMatrix() const;   
    void saveStateDataToCSV();
};

#endif // DRONE_H
