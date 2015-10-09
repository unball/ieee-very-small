/**
 * @file   math.hpp
 * @author Matheus Vieira Portela
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Mathematical functions
 *
 * Implements mathematical methods for general purpose calculations
 */

#include <unball/utils/math.hpp>

/**
 * Saturate a number x, forcing it to the interval (-limit) <= x <= (limit).
 * @param x The number to saturate.
 * @param limit The limit of the saturation interval.
 * @return The saturated number.
 */
float math::saturate(float x, float limit)
{
    if (x > limit)
        x = limit;
    else if (x < -limit)
        x = -limit;
    
    return x;
}

/**
 * Reduce an angle to the interval (-M_PI) <= angle < (M_PI).
 * @param angle The angle to saturate.
 * @return The reduced angle.
 */
float math::reduceAngle(float angle)
{
    while (angle < -M_PI)
        angle += 2*M_PI;
    while (angle > M_PI)
        angle -= 2*M_PI;
        
    return angle;
}

/**
 * Calculate the distance from (x1, y1) to (x2, y2) by using the Pythagorean theorem.
 * @param x1 The x coordinate of the first point.
 * @param y1 The y coordinate of the first point.
 * @param x2 The x coordinate of the second point.
 * @param y2 The y coordinate of the second point.
 * @return The calculated distance.
 */
float math::calculateDistance(float x1, float y1, float x2, float y2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

/**
 * Calculate the distance from point1 to point2 by using the Pythagorean theorem.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The calculated distance.
 */
float math::calculateDistance(Point point1, Point point2)
{
    return calculateDistance(point1.getX(), point1.getY(), point2.getX(), point2.getY());
}

/**
 * Calculate the angle from (x1, y1) to (x2, y2) by using arc-tangent.
 * @param x1 The x coordinate of the first point.
 * @param y1 The y coordinate of the first point.
 * @param x2 The x coordinate of the second point.
 * @param y2 The y coordinate of the second point.
 * @return The calculated angle.
 */
float math::calculateAngle(float x1, float y1, float x2, float y2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    return atan2(dy, dx);
}

/**
 * Calculate the angle from point1 to point2 by using arc-tangent.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The calculated angle.
 */
float math::calculateAngle(Point point1, Point point2)
{
    return calculateAngle(point1.getX(), point1.getY(), point2.getX(), point2.getY());
}

float math::invertAngle(float angle)
{
    if (angle == 0)
        return M_PI;
    else if (angle == M_PI)
        return 0;
    return reduceAngle(angle + M_PI);
}