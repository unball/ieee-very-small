/**
 * @file   perpendicular_potential_field.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   09/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Null Potential field class.
 */

#include <unball/strategy/null_potential_field.hpp>

NullPotentialField::NullPotentialField()
{
}

Vector PerpendicularPotentialField::calculateForce(Vector position)
{
    Vector result;
    float magnitude = 0.0;
    float angle = 0.0;
    result.setPolar(magnitude, angle);
    return result;
}