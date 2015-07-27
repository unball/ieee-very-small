/**
 * @file   radial_potential_field.cpp
 * @author Matheus Vieira Portela
 * @date   27/06/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that affects objects according to the distance to its
 * origin.
 */

#ifndef UNBALL_RADIAL_POTENTIAL_FIELD_H_
#define UNBALL_RADIAL_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class RadialPotentialField : public PotentialField
{
  public:
    RadialPotentialField(Vector origin, float magnitude);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float magnitude_;
};

#endif  // UNBALL_RADIAL_POTENTIAL_FIELD_H_
