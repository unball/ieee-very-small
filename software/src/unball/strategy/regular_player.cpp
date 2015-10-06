#include <unball/strategy/regular_player.hpp>

void RegularPlayer::buildPotentialFields(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));

    if (isInBallRange(robot_number)) 
        potential_fields_.push_back(new SelectivePotentialField(ball_position, 0, M_PI/6, 6));
    else 
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 20));

    for (int i = 1; i < 6; ++i) 
    {
        if (i != robot_number) 
        {
            //potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 0.3));
        }
    }
}

bool RegularPlayer::isInBallRange(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector robot_position(robot[robot_number].getX(), robot[robot_number].getY());
    Vector difference = robot_position - ball_position;    
    
    return difference.getMagnitude() < BALL_RANGE_;
}