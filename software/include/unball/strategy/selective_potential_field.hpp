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
    SelectivePotentialField(Vector origin, float direction, float width, float range);
    Vector calculateForce(Vector position);
  
  private:
    // The position attracting robots.
    Vector origin_;

    // The direction, in radians, of the field normal vector.
    float direction_;

    // The width, in radians, this field has.
    float width_;

    // The magnitude of the force applied to the robots.
    float range_;

    bool isInTheCone(Vector difference);
    Vector applyAttractivePotentialField(Vector difference);
    Vector applyTangentialField(Vector difference);
};

#endif  // UNBALL_SELECTIVE_POTENTIAL_FIELD_H_
