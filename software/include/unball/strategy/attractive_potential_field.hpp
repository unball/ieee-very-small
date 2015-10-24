/**
 * @file   attractive_potential_field.hpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Field that attracts the robot to the desired position.
 */

#ifndef UNBALL_ATTRACTIVE_POTENTIAL_FIELD_H_
#define UNBALL_ATTRACTIVE_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class AttractivePotentialField : public PotentialField
{
  public:
    AttractivePotentialField(Vector origin, float magnitude);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float magnitude_;

	static float const MIN_MAGNITUDE_ = 0.5;    
};

#endif  // UNBALL_ATTRACTIVE_POTENTIAL_FIELD_H_
