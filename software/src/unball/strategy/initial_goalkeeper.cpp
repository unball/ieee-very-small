#include <unball/strategy/initial_goalkeeper.hpp>

float const InitialGoalkeeper::OFFSET = 0.18;

InitialGoalkeeper::InitialGoalkeeper()
{
	behaviour_ = INITIAL_GOALKEEPER;
}

void InitialGoalkeeper::buildPotentialFields(int robot_number)
{
	int sign = fabs(Goals::getInstance().friendly_goal_.getY())/Goals::getInstance().friendly_goal_.getY();
	Vector position(Goals::getInstance().friendly_goal_.getX(), Goals::getInstance().friendly_goal_.getY() - sign*OFFSET);
	potential_fields_.push_back(new AttractivePotentialField(position,5));
}
