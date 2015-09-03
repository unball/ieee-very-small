/**
 * @file   repulsive_potential_field.hpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Field that repels the robot from a position.
 */

#ifndef UNBALL_REPULSIVE_POTENTIAL_FIELD_H_
#define UNBALL_REPULSIVE_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class RepulsivePotentialField : public PotentialField
{
  public:
    RepulsivePotentialField(Vector origin, float range);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float range_;

    bool isInRange(Vector position);
};

#endif  // UNBALL_REPULSIVE_POTENTIAL_FIELD_H_
