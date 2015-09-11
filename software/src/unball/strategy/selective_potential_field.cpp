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
    float width, float range) :
    origin_(origin), direction_(direction), width_(width), range_(range)
{
}

Vector SelectivePotentialField::calculateForce(Vector robot_position)
{
    Vector result;
    Vector difference = robot_position - origin_;
    float magnitude = 0.0;
    float angle = 0.0;

    if (isInTheCone(difference))
        result = applyAttractivePotentialField(difference);
    else
        result = applyTangentialField(difference);
    
    return result;
}

bool SelectivePotentialField::isInTheCone(Vector difference)
{
    return (fabs(difference.getDirection() - direction_) <= width_/2);
}

Vector SelectivePotentialField::applyAttractivePotentialField(Vector difference)
{
    float angle = difference.getDirection();
    float magnitude = difference.getMagnitude()*range_;    
    
    Vector result;
    result.setPolar(magnitude, angle); 
    return result;
}

Vector SelectivePotentialField::applyTangentialField(Vector difference)
{
        float angle = difference.getDirection() - M_PI_2;
        float magnitude = range_;
        
        Vector result;
        result.setPolar(magnitude, angle+0.3);
        return result;
}