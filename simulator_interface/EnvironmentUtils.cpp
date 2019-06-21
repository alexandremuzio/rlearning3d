#include "EnvironmentUtils.h"

#include <random>

Vector2<double> EnvironmentUtils::getRandomPointInRectangle(double x, double y, double maxWidth, double maxHeight,
                                                            double minWidth, double minHeight) {
    // Set initial random robot position
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1, 1);
    double rand;

    rand = dis(gen);
    double xDoubled = (rand > 0)? minWidth + rand * (maxWidth - minWidth)
                :  -minWidth + rand * (maxWidth - minWidth);

    rand = dis(gen);
    double yDoubled = (rand > 0)? minHeight + rand * (maxHeight - minHeight)
                :  -minHeight + rand * (maxHeight - minHeight);


    return Vector2<double>(xDoubled / 2 + x, yDoubled / 2 + y);
}

Vector2<double> EnvironmentUtils::getRandomPointInCircle(double x, double y, double radiusMax, double radiusMin) {
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1, 1);

    double rand;

    rand = dis(gen);
    double radius =  radiusMin + abs(rand) * (radiusMax - radiusMin);

    rand = dis(gen);
    double theta = rand * M_PI;

    double xRes = radius * cos(theta);
    double yRes = radius * sin(theta);

    return Vector2<double>(xRes, yRes);
}


double EnvironmentUtils::getRandomAngle(double maxTheta) {
    // Set initial random robot position
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1, 1);

    double rand = dis(gen);
    return rand * maxTheta;
}


bool EnvironmentUtils::pointInsideArea(double x, double y, double maxX, double maxY) {
    return x < maxX && x > -maxX
           && y < maxY && y > -maxY;
}