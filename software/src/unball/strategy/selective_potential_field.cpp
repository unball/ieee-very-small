/**
 * @file   selective_potential_field.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   29/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Selective Potential field class.
 */

#include <unball/strategy/selective_potential_field.hpp>

SelectivePotentialField::SelectivePotentialField(Vector origin, float direction,
    float width, float magnitude) :
    origin_(origin), direction_(direction), width_(width), magnitude_(magnitude)
{
}

Vector SelectivePotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;
    float magnitude = 0.0;
    float angle = 0.0;

    if ((difference.getDirection() - direction_ <= width_/2)
        and (difference.getDirection() - direction_>= -width_/2))
    {
        angle = difference.getDirection();
        magnitude = difference.getMagnitude()*magnitude_;
    }
    
    result.setPolar(magnitude, angle);
    return result;
}