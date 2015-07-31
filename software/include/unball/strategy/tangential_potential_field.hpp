/**
 * @file   tangential_potential_field.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   29/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that affects objects to be oriented
 * around to the origin according to the distance to its
 * origin.
 */

#ifndef UNBALL_TANGENTIAL_POTENTIAL_FIELD_H_
#define UNBALL_TANGENTIAL_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class TangentialPotentialField : public PotentialField
{
  public:
    TangentialPotentialField(Vector origin, float magnitude);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float magnitude_;
};

#endif  // UNBALL_TANGENTIAL_POTENTIAL_FIELD_H_