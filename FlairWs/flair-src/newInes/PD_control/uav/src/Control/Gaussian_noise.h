#ifndef GAUSSIAN_NOISE_H
#define GAUSSIAN_NOISE_H

#include <Eigen/Dense>

class GaussianNoise {
public:
    GaussianNoise(float meanValue, float stdDev, float rangeMin, float rangeMax);
    Eigen::Vector3f generateNoise(); // Genera ruido para X, Y y Z.

private:
    float meanValue;
    float stdDev;
    float rangeMin;
    float rangeMax;

    float generateSingleNoise(); // Genera ruido para una sola coordenada.
};

#endif // GAUSSIAN_NOISE_H
