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
    float width, float range, bool isSmooth) :
    origin_(origin), direction_(direction), width_(width), range_(range), isSmooth_(isSmooth)
{
}

Vector SelectivePotentialField::calculateForce(Vector robot_position)
{
    Vector result;
    Vector difference = robot_position - origin_;

    if (isInTheCone(difference))
        result += applyAttractivePotentialField(difference);
    else if (isSmooth_ and isInTheCone(difference, 1.5))
    {
        result += applyAttractivePotentialField(difference);
        result += applyTangentialField(difference);
    }
    else
    {
        result += applyTangentialField(difference);   
    }
    
    return result;
}

bool SelectivePotentialField::isInTheCone(Vector difference, float weight)
{
    return (fabs(difference.getDirection() - direction_) <= weight*width_);
}

Vector SelectivePotentialField::applyAttractivePotentialField(Vector difference)
{
    float angle = difference.getDirection();
    float magnitude = difference.getMagnitude()*range_;    
    
    if (magnitude < MIN_MAGNITUDE_)
        magnitude = MIN_MAGNITUDE_;

    Vector result;
    result.setPolar(magnitude, angle); 
    return result;
}

Vector SelectivePotentialField::applyTangentialField(Vector difference)
{
        float angle = difference.getDirection();
        float magnitude = range_;
        
        int angle_quadrant = math::quadrant(angle);
        int direction_quadrant = math::quadrant(direction_);

        if(shouldRotateClockwise(angle_quadrant, direction_quadrant, math::reduceAngle(angle - direction_)))
            angle = rotateClockwise(angle);
        else
            angle = rotateCounterClockwise(angle); 

        Vector result;
        result.setPolar(magnitude, angle);
        return result;
}

bool SelectivePotentialField::shouldRotateClockwise(int angle_quadrant, int direction_quadrant, float resultant_angle)
{
    bool clockwise = true;

    if (direction_quadrant == 1 and angle_quadrant == 2)
        clockwise = true;
    else if (direction_quadrant == 1 and angle_quadrant == 4)
        clockwise = false;
    else if(direction_quadrant == 2 and angle_quadrant == 3)
        clockwise = true;
    else if (direction_quadrant == 2 and angle_quadrant == 1)
        clockwise = false;
    else if (direction_quadrant == 3 and angle_quadrant == 4)
        clockwise = true;
    else if (direction_quadrant == 3 and angle_quadrant == 2)
        clockwise = false;
    else if (direction_quadrant == 4 and angle_quadrant == 1)
        clockwise = true;
    else if (direction_quadrant == 4 and angle_quadrant == 3)
        clockwise = false;
    else if (resultant_angle <= 0)
        clockwise = false;

    return clockwise;
}

float SelectivePotentialField::rotateClockwise(float angle) {
    return (math::reduceAngle(angle + M_PI/2 - TANGENTIAL_CORRECTION_));
}


float SelectivePotentialField::rotateCounterClockwise(float angle) {
    return (math::reduceAngle(angle - M_PI/2 + TANGENTIAL_CORRECTION_));
}