/**
 * @file   parallel_potential_field.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   27/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that affects objects to be oriented parallel
 * to the origin according to the distance to its origin.
 */

#ifndef UNBALL_PARALLEL_POTENTIAL_FIELD_H_
#define UNBALL_PARALLEL_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class ParallelPotentialField : public PotentialField
{
  public:
    ParallelPotentialField(Vector field_force);
    Vector calculateForce(Vector position);
  
  private:
    Vector field_force_;
};

#endif  // UNBALL_PARALLEL_POTENTIAL_FIELD_H_