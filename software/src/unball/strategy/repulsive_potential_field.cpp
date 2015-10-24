/**
 * @file   repulsive_potential_field.cpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Field that repels the robot from a position.
 */

#include <unball/strategy/repulsive_potential_field.hpp>

RepulsivePotentialField::RepulsivePotentialField(Vector origin, float range, float magnitude_weight) :
    origin_(origin), range_(range), magnitude_weight_(magnitude_weight)
{
}

Vector RepulsivePotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = origin_ - position;   
    float angle = difference.getDirection();
 
    float magnitude = 0;
    if (isInRange(position))
 		magnitude = (range_ - difference.getMagnitude())/(range_/magnitude_weight_);

    result.setPolar(magnitude, angle);
   	
    return result;
}

bool RepulsivePotentialField::isInRange(Vector position)
{
	Vector difference = origin_ - position;   
	return difference.getMagnitude() < range_;
}