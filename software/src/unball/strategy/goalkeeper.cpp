#include <unball/strategy/goalkeeper.hpp>

Goalkeeper::Goalkeeper()
{
	behaviour_ = GOALKEEPER;
}

void Goalkeeper::buildPotentialFields(int robot_number)
{
    updateBallPos();
    Vector robot_pos(robot[robot_number].getX(), robot[robot_number].getY());
    Vector ball_line(robot[robot_number].getX(), ball_pos_.getY());

    if (isBallBelowGoalkeeper(robot_number))
    {
    	if (isBallInRange())
			potential_fields_.push_back(new AttractivePotentialField(ball_line, 20));
		else 
			stayAtTheBoundary();
    }
    //else
    //{
    	//Danger: kick the ball as far as possible
    //}
}

void Goalkeeper::updateBallPos()
{
	Vector ball_pos(Ball::getInstance().getX(),Ball::getInstance().getY());
	ball_pos_ = ball_pos;
}

bool Goalkeeper::isBallBelowGoalkeeper(int goalkeeper_number)
{
	return (ball_pos_.getX() < robot[2].getX());
}

bool Goalkeeper::isBallInRange()
{
	return (ball_pos_.getX() > RIGHT_LIMIT and ball_pos_.getX() < LEFT_LIMIT);
}

void Goalkeeper::stayAtTheBoundary()
{
	if (ball_pos_.getY() < RIGHT_LIMIT)
		goToTheBoundary(RIGHT_LIMIT);
	else if (ball_pos_.getY() > LEFT_LIMIT)
		goToTheBoundary(LEFT_LIMIT);
}

void Goalkeeper::goToTheBoundary(float limit)
{
	clearPotentialFields();
	potential_fields_.push_back(new AttractivePotentialField(Vector(robot[2].getX(),limit),20));
}