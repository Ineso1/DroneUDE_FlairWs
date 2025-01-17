/*
#ifndef TRAYECTORY_H
#define TRAYECTORY_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <Vector3Df>

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
*/
#ifndef TRAYECTORY_H
#define TRAYECTORY_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <Vector3D.h>

class Trayectory {
public:
    Trayectory();
    Trayectory(float diameter, float fixedZ, float resolution);
    ~Trayectory();

    // Genera el siguiente punto de la trayectoria
    flair::core::Vector3Df getNextPoint();

private:
    float diameter;     // Diámetro del círculo
    float fixedZ;       // Altura fija (coordenada Z)
    float resolution;   // Resolución (distancia entre puntos)
    float radius;       // Radio del círculo
    float circumference; // Circunferencia del círculo
    size_t numPoints;   // Número total de puntos en la trayectoria
    size_t currentIndex; // Índice del punto actual
};
#endif // TRAYECTORY_H

