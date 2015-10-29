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
    {
        //potential_fields_.push_back(new SelectivePotentialField(ball_position, 
        //    kick_target_.getDirection(), M_PI/4, 6));
        potential_fields_.push_back(new SelectivePotentialField(ball_position, 
            kick_target_.getDirection(), M_PI/4, 6, false));
    }
    else
    {
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 6));
    }

    /*potential_fields_.push_back(new RepulsivePotentialField(
        Vector(robot[Goals::getInstance().findOpponentGoalkeeper()].getX(),
            robot[Goals::getInstance().findOpponentGoalkeeper()].getY()), 0.3));

    for (int i=0; i<6; i++)
    {
        if (i != robot_number)
            potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 0.3, 3));    
    }
    potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[2].getX(), robot[2].getY()), 0.3, 5));
    if (friendly_kicker_ != -1)
    	potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[friendly_kicker_].getX(),
    																   robot[friendly_kicker_].getY()), 0.3, 5));
    avoidTheWalls(robot_number);*/
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

void AssistentPlayer::avoidTheWalls(int robot_number)
{
    if (robot[robot_number].getY() > 0.55)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getX(), robot[robot_number].getY()),
        Vector(robot[robot_number].getX(), 0.65), 0.2));
    else if (robot[robot_number].getY() < -0.55)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getX(), robot[robot_number].getY()),
        Vector(robot[robot_number].getX(), -0.65), 0.2));
    else if (robot[robot_number].getX() > 0.65)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getX(), robot[robot_number].getY()),
        Vector(0.75, robot[robot_number].getY()), 0.2));
    else if (robot[robot_number].getX() < -0.65)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getX(), robot[robot_number].getY()),
        Vector(-0.75, robot[robot_number].getY()), 0.2));
}