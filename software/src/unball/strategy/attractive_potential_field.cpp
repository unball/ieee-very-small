/**
 * @file   attractive_potential_field.cpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Field that attracts the robot to the desired position.
 */

#include <unball/strategy/attractive_potential_field.hpp>

AttractivePotentialField::AttractivePotentialField(Vector origin, float magnitude) :
    origin_(origin), magnitude_(magnitude)
{
}

Vector AttractivePotentialField::calculateForce(Vector position)
{
    Vector result;
    Vector difference = position - origin_;

	ROS_ERROR("[Attractive]positionAngle = %.2f, originAngle = %.2f, differenceAngle = %.2f", 
		position.getDirection()*180/M_PI,origin_.getDirection()*180/M_PI, difference.getDirection()*180/M_PI);

	ROS_ERROR("[Attractive]positionMagnitude = %.2f, originMagnitude = %.2f, differenceMagnitude = %.2f", 
		position.getMagnitude(),origin_.getMagnitude(), difference.getMagnitude());


    float angle = difference.getDirection();
    float magnitude = difference.getMagnitude()*magnitude_;

    if (magnitude < MIN_MAGNITUDE_)
        magnitude = MIN_MAGNITUDE_;
    
    result.setPolar(magnitude, angle);

	ROS_ERROR("[Attractive]angle = %.2f, magnitude = %.2f", angle*180/M_PI,magnitude);
	ROS_ERROR("[Attractive]result angle = %.2f, result magnitude = %.2f", 
		result.getDirection()*180/M_PI,result.getMagnitude());

    return result;
}