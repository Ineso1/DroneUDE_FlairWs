#include "Gaussian_noise.h"
#include <random>
#include <limits>
#include "iostream"

GaussianNoise::GaussianNoise(float meanValue, float stdDev, float rangeMin, float rangeMax)
    : meanValue(meanValue), stdDev(stdDev), rangeMin(rangeMin), rangeMax(rangeMax) {
    std::cout << "Se ha generado el ruido gaussiano\n  ( ◡̀_◡́)ᕤ\n\U0001f514\n";
    }

float GaussianNoise::generateSingleNoise() {
    std::random_device rd; // Semilla aleatoria.
    std::mt19937 gen(rd()); // Generador Mersenne Twister.
    std::normal_distribution<float> dist(meanValue, stdDev);

    float value;
    do {
        value = dist(gen); // Genera un valor con distribución normal.
    } while (value < rangeMin || value > rangeMax);

    return value;
}

Eigen::Vector3f GaussianNoise::generateNoise() {
    Eigen::Vector3f noise;
    noise.x() = generateSingleNoise(); // Ruido en X.
    noise.y() = generateSingleNoise(); // Ruido en Y.
    noise.z() = generateSingleNoise(); // Ruido en Z.
    return noise;
}
