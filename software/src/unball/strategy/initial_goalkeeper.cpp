#include <unball/strategy/initial_goalkeeper.hpp>

InitialGoalkeeper::InitialGoalkeeper()
{
	behaviour_ = INITIAL_GOALKEEPER;
}

void InitialGoalkeeper::buildPotentialFields(int robot_number)
{
	potential_fields_.push_back(new AttractivePotentialField(Goals::getInstance().friendly_goal_,10));
}