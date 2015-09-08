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
}

Vector Player::calculateResultantForce(int robot_number)
{
    Vector resultant_force;
    Vector position(robot[robot_number].getX(), robot[robot_number].getY());

    for (int i = 0; i < potential_fields_.size(); ++i) 
        resultant_force += potential_fields_[i]->calculateForce(position);
    return resultant_force;	
}