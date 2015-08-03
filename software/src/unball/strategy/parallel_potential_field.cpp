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

ParallelPotentialField::ParallelPotentialField(Vector field_force) :
    field_force_(field_force)
{
}

Vector ParallelPotentialField::calculateForce(Vector position)
{
    return field_force_;
}