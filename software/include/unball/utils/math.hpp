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

#ifndef MATH_H_
#define MATH_H_

#include <cmath>

namespace math
{
    float saturate(float x, float limit);
    float reduceAngle(float angle);
    float calculateDistance(float x1, float y1, float x2, float y2);
    float calculateAngle(float x1, float y1, float x2, float y2);
}

#endif // MATH_H_