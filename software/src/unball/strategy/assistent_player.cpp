#include <unball/strategy/assistent_player.hpp>


AssistentPlayer::AssistentPlayer()
{
	behaviour_ = ASSISTENT_PLAYER;
	friendly_kicker_ = -1;
}

AssistentPlayer::AssistentPlayer(int friendly_kicker)
{
	behaviour_ = ASSISTENT_PLAYER;
	friendly_kicker_ = friendly_kicker;
}

void AssistentPlayer::buildPotentialFields(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector difference;

    findTarget();

	if (isInBallRange(robot_number))
        potential_fields_.push_back(new SelectivePotentialField(ball_position, kick_target_.getDirection(), M_PI/4, 6));
    else
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 20));

    potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[5].getX(), robot[5].getY()), 0.3));

    for (int i=0; i<6; i++)
    {
        if (i != robot_number)
            potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 0.3, 3));    
    }

    if (friendly_kicker_ != -1)
    	potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[friendly_kicker_].getX(),
    																   robot[friendly_kicker_].getY()), 0.3, 5));
}

bool AssistentPlayer::isInBallRange(int robot_number)
{
	Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector robot_position(robot[robot_number].getX(), robot[robot_number].getY());
    Vector difference = robot_position - ball_position;    
    
    return difference.getMagnitude() < BALL_RANGE_;
}

void AssistentPlayer::findTarget()
{
	kick_target_ = Vector(0,0) - Goals::getInstance().opponent_goal_;
}