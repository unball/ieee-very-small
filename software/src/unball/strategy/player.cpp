#include <unball/strategy/player.hpp>

Player::~Player()
{
	clearPotentialFields();
}

void Player::clearPotentialFields()
{
    for (int i = potential_fields_.size()-1; i >= 0; --i)
        delete potential_fields_[i];
    potential_fields_.clear();
    if (potential_fields_.size() == 0)
    	ROS_ERROR("[Player]clearPotentialFields: cleared all potential fields");
   	else
    	ROS_ERROR("[Player]clearPotentialFields: did not clear all potential fields");
}

Vector Player::calculateResultantForce(int robot_number)
{
	ROS_ERROR("[Player]calculateResultantForce:");
    Vector resultant_force;
    Vector position(robot[robot_number].getX(), robot[robot_number].getY());

    for (int i = 0; i < potential_fields_.size(); ++i) 
        resultant_force += potential_fields_[i]->calculateForce(position);
    return resultant_force;	
}