/*
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
*/
#include "Trayectory.h"

Trayectory::Trayectory()
    : currentIndex(0), diameter(diameter), fixedZ(fixedZ), resolution(resolution) 
{
}

Trayectory::Trayectory(float diameter, float fixedZ, float resolution)
    : currentIndex(0), diameter(diameter), fixedZ(fixedZ), resolution(resolution) 
{
    // Calcular los parámetros para la trayectoria circular
    radius = diameter / 2.0f;
    circumference = 2.0f * M_PI * radius;
    numPoints = static_cast<size_t>(circumference / resolution);

    std::cout << "┻┳|\n┳┻|\n┻┳|•.•). I'm trayectorying.....\n┳┻|⊂ﾉ\n┻┳\n";
}

Trayectory::~Trayectory() {}

flair::core::Vector3Df Trayectory::getNextPoint() {
    // Calcular el punto siguiente en la trayectoria
    float angle = static_cast<float>(currentIndex) * 2.0f * M_PI / static_cast<float>(numPoints);
    float x = radius * std::cos(angle);
    float y = radius * std::sin(angle);

    // Actualizar el índice para el próximo punto
    currentIndex = (currentIndex + 1) % numPoints;

    // Retornar el punto calculado
    //return Eigen::Vector3f(x, y, fixedZ);
    return flair::core::Vector3Df(x, y, fixedZ);
}
