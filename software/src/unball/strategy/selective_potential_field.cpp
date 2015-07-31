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

SelectivePotentialField(Vector origin, float magnitude, float width) :
    origin_(origin), magnitude_(magnitude), width_(width)
{
}

Vector SelectivePotentialField::calculateForce(Vector position)
{
    Vector difference = position - origin_;
    float magnitude = 0.0;
    float angle = 0.0;
    if(difference.getMagnitude() < magnitude_)
    {
        if(difference.getDirection() > origin_.getDirection() + (width_/2)
            or difference.getDirection() < origin_.getDirection() - (width_/2))
        { // this potential field acts as a tangential potential field
        
            magnitude = magnitude_;
            if(difference.getDirection() < 0)
               angle = origin_.getDirection() + (M_PI/2);
            else
                angle = origin_.getDirection() - (M_PI/2);
        }else//this potential field acts as a attractive potential field
        { 
            magnitude = magnitude_/difference.getMagnitude();
            angle = difference.getDirection();
        }
    }
    Vector result;
    result.setPolar(magnitude, angle);
    return result;
}