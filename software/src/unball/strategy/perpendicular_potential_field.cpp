/**
 * @file   perpendicular_potential_field.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   27/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Perpendicular Potential field class.
 */

#include <unball/strategy/perpendicular_potential_field.hpp>

PerpendicularPotentialField::PerpendicularPotentialField(Vector origin,
    float magnitude) : origin_(origin), magnitude_(magnitude)
{
}

Vector PerpendicularPotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = origin_ - position;
    float magnitude = 0.0;
    float angle = 0.0;

    if (difference.getMagnitude() <= magnitude_)
    {
        magnitude = magnitude_/difference.getMagnitude();

        if (difference.getDirection() < 0)
    	   angle = origin_.getDirection() + (M_PI/2);
        else
    	   angle = origin_.getDirection() - (M_PI/2);
    }
    
    result.setPolar(magnitude, angle);
    return result;
}