#ifndef ITANDROIDS_SOCCER3D_CPP_ENVIRONMENTUTILS_H
#define ITANDROIDS_SOCCER3D_CPP_ENVIRONMENTUTILS_H

#include "math/Vector2.h"

using itandroids_lib::math::Vector2;

class EnvironmentUtils {
public:

    /// Computes a random position between two rectangles.
    /// \param x Center x coordinate of rectangle
    /// \param y Center y coordinate of rectangle
    /// \param maxWidth
    /// \param maxHeight
    /// \param minWidth
    /// \param minHeight
    static Vector2<double>
    getRandomPointInRectangle(double x, double y, double maxWidth, double maxHeight, double minWidth = 0.0,
                              double minHeight = 0.0);

    /// Computes a random position between two coencentric circles.
    /// \param x Center x coordinate of rectangle
    /// \param y Center y coordinate of rectangle
    /// \param radiusMax Greater radius
    /// \param maxHeight Smaller radius (optional)
    static Vector2<double>
    getRandomPointInCircle(double x, double y, double radiusMax, double radiusMin = 0.0);

    /// Computes a random angle.
    /// \param maxTheta belong to [-maxTheta, +maxTheta]
    static double getRandomAngle(double maxTheta);

    /// Checks if a point is inside a (0,0) centered rectange.
    static bool pointInsideArea(double x, double y, double maxX, double maxY);
};


#endif //ITANDROIDS_SOCCER3D_CPP_ENVIRONMENTUTILS_H
