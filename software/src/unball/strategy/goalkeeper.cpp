#include <unball/strategy/goalkeeper.hpp>

void Goalkeeper::buildPotentialFields(int robot_number)
{
    Vector ball_line(robot[robot_number].getX(), Ball::getInstance().getY());
    
    potential_fields_.push_back(new AttractivePotentialField(ball_line, 20));
    potential_fields_.push_back(new ParallelPotentialField(Vector(0, 0.6), Vector(0, -10), 0.8));
    potential_fields_.push_back(new ParallelPotentialField(Vector(0, -0.6), Vector(0, 10), 0.4));
}