#include <unball/strategy/kicker_player.hpp>

KickerPlayer::KickerPlayer()
{
    behaviour_ = KICKER_PLAYER;
}

void KickerPlayer::buildPotentialFields(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector difference;

    findTarget();

    difference = ball_position - kick_target_;

    if (isInBallRange(robot_number))
        potential_fields_.push_back(new SelectivePotentialField(ball_position, difference.getDirection(), M_PI/4, 6));
    else
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 20));

    potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[5].getX(), robot[5].getY()), 0.3, 0.9));
}

void KickerPlayer::findTarget()
{
    target_ = 0;
    
    if(opponentGoalkeeperIsInGoalRange(5))
    {
        if(robot[5].getY() > 0)
            target_ = robot[5].getY() - (0.2 + fabs(robot[5].getY())/2);
        else
            target_ = robot[5].getY() - (0.2 + fabs(robot[5].getY())/2);
    }

    kick_target_ = Vector(Goals::getInstance().opponent_goal_.getX(), target_);
}

bool KickerPlayer::isInBallRange(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector robot_position(robot[robot_number].getX(), robot[robot_number].getY());
    Vector difference = robot_position - ball_position;    
    
    return difference.getMagnitude() < BALL_RANGE_;
}

bool KickerPlayer::opponentGoalkeeperIsInGoalRange(int opponent_goalkeeper)
{
    return (robot[opponent_goalkeeper].getY() > -0.22 and robot[opponent_goalkeeper].getY() < 0.22);
}