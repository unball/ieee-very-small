/**
 * @file   ball.cpp
 * @author Matheus Vieira Portela
 * @date   01/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Ball class
 *
 * Implements ball methods for strategy
 */

#include <unball/strategy/ball.hpp>
#include <cmath>
#include <ros/ros.h>

/**
 * Ball instance.
 */
Ball* Ball::instance = NULL;

Ball::Ball()
{
    pos_.set(0,0);
    vel_.set(0,0);
}

Ball& Ball::getInstance()
{
    if (instance == NULL)
        instance = new Ball();

    return *instance;
}

Point Ball::getPos()
{
    return(pos_);
}

float Ball::getX()
{
    return pos_.getX();
}

float Ball::getY()
{
    return pos_.getY();
}

float Ball::getVelX()
{
    return vel_.getX();
}

float Ball::getVelY()
{
    return vel_.getY();
}

float Ball::getVelAbs()
{
    return (sqrt(pow(vel_.getX(),2)+pow(vel_.getY(),2)));
}

float Ball::getVelAngle()
{
    return (Point().findAngle(pos_));
}

void Ball::setPosition(float x, float y)
{
    pos_.set(x,y);
}

void Ball::setVel(float vel_x, float vel_y)
{
    vel_.set(vel_x,vel_y);
}

/**
 * Update  the ball (x, y) position and all internal state variables of the ball, such as velocities.
 * Angular velocity is set in rad/s.
 * @param x x coordinate, in centimeters.
 * @param y y coordinate, in centimeters.
 */
void Ball::update(float x, float y)
{
    ROS_DEBUG("Updating ball position");
    
    // calculate velocities
    setVel(x - getX(), y - getY());
    
    // refresh position
    setPosition(x, y);
    
    ROS_DEBUG("Ball X = %f", getX());
    ROS_DEBUG("Ball Y = %f", getY());
    ROS_DEBUG("Ball Vel X = %f", getVelX());
    ROS_DEBUG("Ball Vel Y = %f", getVelY());
    ROS_DEBUG("Ball Vel Abs = %f", getVelAbs());
    ROS_DEBUG("Ball Vel Angle = %f", getVelAngle()*180.0/M_PI); // printing in deg/s
}
