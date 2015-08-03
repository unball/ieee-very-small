/**
 * @file   tangential_potential_field.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   29/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Tangential Potential field class.
 */

#include <unball/strategy/tangential_potential_field.hpp>

TangentialPotentialField::TangentialPotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector TangentialPotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;
    float magnitude = 0.0;
    float angle = 0.0;

    if (difference.getMagnitude() <= magnitude_)
    {
        magnitude = magnitude_;
        
        if (difference.getDirection() < 0)
           angle = origin_.getDirection() + (M_PI/2);
        else
           angle = origin_.getDirection() - (M_PI/2);
    }
    
    result.setPolar(magnitude, angle);
    return result;
}