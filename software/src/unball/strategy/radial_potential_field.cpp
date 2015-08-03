/**
 * @file   vector.cpp
 * @author Matheus Vieira Portela
 * @date   27/06/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Two-dimensional vector class.
 */

#include <unball/strategy/radial_potential_field.hpp>

RadialPotentialField::RadialPotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector RadialPotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;
    float magnitude = magnitude_/difference.getMagnitude();
    float angle = difference.getDirection();
    result.setPolar(magnitude, angle);
    return result;
}