#include <unball/strategy/kicker_player.hpp>

void KickerPlayer::buildPotentialFields(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector opponent_goalkeeper(Vector(robot[6].getY(), robot[6].getX()));
    Vector difference;
    // fazer o angulo ser em relacao a posicao do robo, pra desviar do goleiro e fazer gol
    if(robot[6].getY() > 0)
        difference = ball_position - Goals::getInstance().opponent_goal_ - opponent_goalkeeper;
    else
        difference = ball_position - Goals::getInstance().opponent_goal_ + opponent_goalkeeper;

    if (isInBallRange(robot_number))
        potential_fields_.push_back(new SelectivePotentialField(ball_position, difference.getDirection(), M_PI/5, 6));
    else
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 20));

    for (int i = 1; i < 6; ++i) 
    {
        if (i != robot_number) 
        {
            potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 0.3));
        }
    }
}

bool KickerPlayer::isInBallRange(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector robot_position(robot[robot_number].getX(), robot[robot_number].getY());
    Vector difference = robot_position - ball_position;    
    
    return difference.getMagnitude() < BALL_RANGE_;
}