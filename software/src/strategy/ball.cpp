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
 * Ball global object
 */
Ball ball;

Ball::Ball()
{
    x_ = 0;
    y_ = 0;
    vel_x_ = 0;
    vel_y_ = 0;
    vel_abs_ = 0;
    vel_angle_ = 0;
}

float Ball::getX()
{
    return x_;
}

float Ball::getY()
{
    return y_;
}

float Ball::getVelX()
{
    return vel_x_;
}

float Ball::getVelY()
{
    return vel_y_;
}

float Ball::getVelAbs()
{
    return vel_abs_;
}

float Ball::getVelAngle()
{
    return vel_angle_;
}

void Ball::setPosition(float x, float y)
{
    x_ = x;
    y_ = y;
}

void Ball::setVel(float vel_x, float vel_y)
{
    vel_x_ = vel_x;
    vel_y_ = vel_y;
}

void Ball::setVelAbs(float vel_abs)
{
    vel_abs_ = vel_abs;
}

void Ball::setVelAngle(float vel_angle)
{
    vel_angle_ = vel_angle;
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
    setVelAbs(sqrt(pow(getVelX(), 2) + pow(getVelY(), 2)));
    setVelAngle(atan2(y - getY(), x - getX()));
    
    // refresh position
    setPosition(x, y);
    
    ROS_DEBUG("Ball X = %f", getX());
    ROS_DEBUG("Ball Y = %f", getY());
    ROS_DEBUG("Ball Vel X = %f", getVelX());
    ROS_DEBUG("Ball Vel Y = %f", getVelY());
    ROS_DEBUG("Ball Vel Abs = %f", getVelAbs());
    ROS_DEBUG("Ball Vel Angle = %f", getVelAngle()*180.0/M_PI); // printing in deg/s
}
