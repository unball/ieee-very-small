#include <unball/strategy/initial_goalkeeper.hpp>

InitialGoalkeeper::InitialGoalkeeper()
{
	behaviour_ = INITIAL_GOALKEEPER;
}

void InitialGoalkeeper::buildPotentialFields(int robot_number)
{
	int sign = fabs(Goals::getInstance().friendly_goal_.getX())/Goals::getInstance().friendly_goal_.getX();
	
	Vector position(Goals::getInstance().friendly_goal_.getX() - sign*OFFSET, Goals::getInstance().friendly_goal_.getY());
	potential_fields_.push_back(new AttractivePotentialField(position,10));
}