/**
 * @file   selective_potential_field.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   29/07/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Potential field that affects objects according to the distance to its
 * origin in a particular area and with a specific width. It consist of a junction
 * of the radial and tangencial field, but with some particularities. 
 */

#ifndef UNBALL_SELECTIVE_POTENTIAL_FIELD_H_
#define UNBALL_SELECTIVE_POTENTIAL_FIELD_H_

#include <unball/utils/vector.hpp>
#include <unball/strategy/potential_field.hpp>

class SelectivePotentialField : public PotentialField
{
  public:
    SelectivePotentialField(Vector origin, float magnitude, float width);
    Vector calculateForce(Vector position);
  
  private:
    Vector origin_;
    float magnitude_;
    float width_;

};

#endif  // UNBALL_SELECTIVE_POTENTIAL_FIELD_H_
