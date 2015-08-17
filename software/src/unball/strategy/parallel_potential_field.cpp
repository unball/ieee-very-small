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

ParallelPotentialField::ParallelPotentialField(Vector origin,
    Vector field_force, float max_distance) :
    origin_(origin), field_force_(field_force), max_distance_(max_distance)
{
}

Vector ParallelPotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;
    difference.project(field_force_);

    return result;
}