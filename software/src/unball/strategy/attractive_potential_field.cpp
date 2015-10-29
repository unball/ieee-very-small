/**
 * @file   attractive_potential_field.cpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Field that attracts the robot to the desired position.
 */

#include <unball/strategy/attractive_potential_field.hpp>

AttractivePotentialField::AttractivePotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector AttractivePotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;

    float angle = difference.getDirection();
    float magnitude = difference.getMagnitude()*magnitude_;

    if (magnitude < MIN_MAGNITUDE_)
        magnitude = MIN_MAGNITUDE_;
    
    result.setPolar(magnitude, angle);

    return result;
}