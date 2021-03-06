#include <unball/strategy/regular_player.hpp>

float const RegularPlayer::BALL_RANGE_ = 0.5;

RegularPlayer::RegularPlayer()
{
    behaviour_ = REGULAR_PLAYER;
}

void RegularPlayer::buildPotentialFields(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));

    if (isInBallRange(robot_number))
        potential_fields_.push_back(new SelectivePotentialField(ball_position, -3*M_PI/4, M_PI/5, 6));
    else
        potential_fields_.push_back(new AttractivePotentialField(ball_position, 20));

    for (int i = 1; i < 6; ++i)
    {
        if (i != robot_number)
        {
            potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 0.3));
        }
    }
    avoidTheWalls(robot_number);
}

bool RegularPlayer::isInBallRange(int robot_number)
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    Vector robot_position(robot[robot_number].getX(), robot[robot_number].getY());
    Vector difference = robot_position - ball_position;

    return difference.getMagnitude() < BALL_RANGE_;
}

void RegularPlayer::avoidTheWalls(int robot_number)
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
