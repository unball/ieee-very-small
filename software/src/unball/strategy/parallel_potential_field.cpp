/**
 * @file   vector.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   27/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Parallel potencial field class.
 */

#include <unball/strategy/parallel_potential_field.hpp>

ParallelPotentialField::ParallelPotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector ParallelPotentialField::calculateForce(Vector position)
{
    Vector difference = position - origin_;
    float magnitude = 0.0;
    float angle = 0.0;
    if(difference.getMagnitude() <= magnitude_)
    {
    	magnitude = magnitude_/difference.getMagnitude();
    	angle = origin_.getDirection();
    }
    Vector result;
    result.setPolar(magnitude, angle);
    return result;
}