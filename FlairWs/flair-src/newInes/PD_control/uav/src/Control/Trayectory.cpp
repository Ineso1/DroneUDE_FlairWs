#include "Trayectory.h"
#include "iostream"
#include <cmath>

Trayectory::Trayectory(float diameter, float fixedZ, float resolution)
    : currentIndex(0) 
{
    // Generate circular trajectory
    float radius = diameter / 2.0f;
    float circumference = 2.0f * M_PI * radius;
    size_t numPoints = static_cast<size_t>(circumference / resolution);

    for (size_t i = 0; i < numPoints; ++i) {
        float angle = static_cast<float>(i) * 2.0f * M_PI / static_cast<float>(numPoints);
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        trajectoryPoints.emplace_back(x, y, fixedZ);
    }
    std::cout << "┻┳|\n┳┻|\n┻┳|•.•). I'm trayectorying.....\n┳┻|⊂ﾉ\n┻┳\n";
}

Trayectory::~Trayectory() {}

Eigen::Vector3f Trayectory::getNextPoint() {
    // Ensure the index wraps around to create a looping trajectory
    Eigen::Vector3f nextPoint = trajectoryPoints[currentIndex];
    currentIndex = (currentIndex + 1) % trajectoryPoints.size();
    return nextPoint;
}