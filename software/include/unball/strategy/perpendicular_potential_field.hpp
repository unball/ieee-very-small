/**
 * @file   perpendicular_potential_field.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   27/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that affects objects to be oriented perpedicular
 * to the origin according to the distance to its origin.
 */

#ifndef UNBALL_PERPENDICULAR_POTENTIAL_FIELD_H_
#define UNBALL_PERPENDICULAR_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class PerpendicularPotentialField : public PotentialField
{
  public:
    PerpendicularPotentialField(Vector origin, float magnitude);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float magnitude_;
};

#endif  // UNBALL_PERPENDUCULAR_POTENTIAL_FIELD_H_