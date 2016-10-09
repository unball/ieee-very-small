/**
 * @file   null_potential_field.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   09/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that that has null infuence on the robot
*/

#ifndef UNBALL_NULL_POTENTIAL_FIELD_H_
#define UNBALL_NULL_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class NullPotentialField : public PotentialField
{
  public:
    NullPotentialField();
    Vector calculateForce(Vector position);
};

#endif  // UNBALL_NULL_POTENTIAL_FIELD_H_