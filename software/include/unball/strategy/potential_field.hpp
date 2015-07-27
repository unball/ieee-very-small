/**
 * @file   potential_field.hpp
 * @author Matheus Vieira Portela
 * @date   27/06/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Abstract potential field.
 */

#ifndef UNBALL_POTENTIAL_FIELD_H_
#define UNBALL_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>

class PotentialField
{
  public:
    Vector calculateForce(Vector position) = 0;
};

#endif  // UNBALL_POTENTIAL_FIELD_H_
