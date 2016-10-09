#include <unball/strategy/goalkeeper_kicker.hpp>

float const GoalkeeperKicker::LEFT_LIMIT = 0.18;
float const GoalkeeperKicker::RIGHT_LIMIT = -0.18;

GoalkeeperKicker::GoalkeeperKicker()
{
	behaviour_ = GOALKEEPER_KICKER;
}

void GoalkeeperKicker::buildPotentialFields(int robot_number)
{
    updateBallPos();
    potential_fields_.push_back(new AttractivePotentialField(ball_pos_, 20));
}

void GoalkeeperKicker::updateBallPos()
{
	Vector ball_pos(Ball::getInstance().getX(),Ball::getInstance().getY());
	ball_pos_ = ball_pos;
}
