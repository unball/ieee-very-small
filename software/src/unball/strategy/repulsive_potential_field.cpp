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

RepulsivePotentialField::RepulsivePotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector RepulsivePotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = origin_ - position;
    float angle = difference.getDirection();
    float magnitude = difference.getMagnitude()/magnitude_;

    result.setPolar(magnitude, angle);
    return result;
}