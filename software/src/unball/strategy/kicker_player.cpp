#include <unball/strategy/kicker_player.hpp>

float const KickerPlayer::BALL_RANGE_ = 0.3;

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
    {
        potential_fields_.push_back(new SelectivePotentialField(ball_position, difference.getDirection(),
            M_PI/4, 6));
    }
    else
    {
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 6));
    }

    //potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[5].getX(), robot[5].getY()), 0.3, 0.9));
    avoidTheWalls(robot_number);
}

void KickerPlayer::findTarget()
{
    target_ = 0;
    int opponent_goalkeeper_index = Goals::getInstance().findOpponentGoalkeeper();

    if(opponentGoalkeeperIsInGoalRange(opponent_goalkeeper_index))
    {
        if(robot[opponent_goalkeeper_index].getX() > 0)
            target_ = robot[opponent_goalkeeper_index].getX() - (0.2 + fabs(robot[opponent_goalkeeper_index].getX())/2);
        else
            target_ = robot[opponent_goalkeeper_index].getX() - (0.2 + fabs(robot[opponent_goalkeeper_index].getX())/2);
    }

    kick_target_ = Vector(target_, Goals::getInstance().opponent_goal_.getY());
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
    return (robot[opponent_goalkeeper].getX() > -0.22 and robot[opponent_goalkeeper].getX() < 0.22);
}

void KickerPlayer::avoidTheWalls(int robot_number)
{
    if (robot[robot_number].getX() > 0.55)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getY(), robot[robot_number].getX()),
        Vector(robot[robot_number].getY(), 0.65), 0.2));
    else if (robot[robot_number].getX() < -0.55)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getY(), robot[robot_number].getX()),
        Vector(robot[robot_number].getY(), -0.65), 0.2));
    else if (robot[robot_number].getY() > 0.65)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getY(), robot[robot_number].getX()),
        Vector(0.75, robot[robot_number].getX()), 0.2));
    else if (robot[robot_number].getY() < -0.65)
        potential_fields_.push_back(new ParallelPotentialField(Vector(robot[robot_number].getY(), robot[robot_number].getX()),
        Vector(-0.75, robot[robot_number].getX()), 0.2));
}
