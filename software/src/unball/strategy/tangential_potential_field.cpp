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
    float angle = difference.getDirection() + M_PI_2;
    result.setPolar(magnitude_, angle);
    return result;
}