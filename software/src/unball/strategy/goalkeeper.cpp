#include <unball/strategy/goalkeeper.hpp>

float const Goalkeeper::LEFT_LIMIT = 0.18;
float const Goalkeeper::RIGHT_LIMIT = -0.18;

Goalkeeper::Goalkeeper()
{
	behaviour_ = GOALKEEPER;
	y_pos_ = robot[2].getY();
}

void Goalkeeper::buildPotentialFields(int robot_number)
{
    updateBallPos();
    Vector robot_pos(robot[robot_number].getX(),y_pos_);
    Vector ball_line(ball_pos_.getX(),robot[robot_number].getY());

    if (isBallInRange())
		potential_fields_.push_back(new AttractivePotentialField(ball_line, 15));
	else
    	stayAtTheBoundary();
}

void Goalkeeper::updateBallPos()
{
	Vector ball_pos(Ball::getInstance().getX(),Ball::getInstance().getY());
	ball_pos_ = ball_pos;
}

bool Goalkeeper::isBallInRange()
{
	return (ball_pos_.getX() > RIGHT_LIMIT and ball_pos_.getX() < LEFT_LIMIT);
}

void Goalkeeper::stayAtTheBoundary()
{
	if (ball_pos_.getX() < RIGHT_LIMIT)
		goToTheBoundary(RIGHT_LIMIT);
	else if (ball_pos_.getX() > LEFT_LIMIT)
		goToTheBoundary(LEFT_LIMIT);
}

void Goalkeeper::goToTheBoundary(float limit)
{
	clearPotentialFields();
	potential_fields_.push_back(new AttractivePotentialField(Vector(limit,robot[2].getY()),5));
}
