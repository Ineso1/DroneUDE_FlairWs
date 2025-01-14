#ifndef TRAYECTORY_H
#define TRAYECTORY_H

#include <vector>
#include <Eigen/Dense>

class Trayectory {
public:
    Trayectory(float diameter, float fixedZ, float resolution);
    ~Trayectory();

    // Generates the next point in the trajectory
    Eigen::Vector3f getNextPoint();

private:
    std::vector<Eigen::Vector3f> trajectoryPoints; // Precomputed trajectory points
    size_t currentIndex;                           // Current index in trajectory
};

#endif // TRAYECTORY_H
