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

    if (isInTheCone(difference))
        result = applyAttractivePotentialField(difference);
    else
        result = applyTangentialField(difference);
    ROS_ERROR("RESULTANT FORCE:\tangle: %f\tmagnitude: %f", result.getDirection()*(180/M_PI), result.getMagnitude());
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
        float angle = difference.getDirection();
        float magnitude = range_;
        
        if(fabs(angle) <= M_PI)
            angle += M_PI_2;
        else
            angle -= M_PI_2;
        Vector result;
        result.setPolar(magnitude, angle);
        return result;
}