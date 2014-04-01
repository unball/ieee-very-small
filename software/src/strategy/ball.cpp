/**
 * @file   ball.cpp
 * @author Matheus Vieira Portela
 * @date   01/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Ball class
 *
 * Implements ball methods for strategy
 */

#include "ball.hpp"
#include <cmath>
#include <ros/ros.h>

Ball::Ball()
{
    this->x_ = 0;
    this->y_ = 0;
    this->vel_x_ = 0;
    this->vel_y_ = 0;
    this->vel_abs_ = 0;
    this->vel_angle_ = 0;
}

float Ball::getX()
{
    return this->x_;
}

float Ball::getY()
{
    return this->y_;
}

float Ball::getVelX()
{
    return this->vel_x_;
}

float Ball::getVelY()
{
    return this->vel_y_;
}

float Ball::getVelAbs()
{
    return this->vel_abs_;
}

float Ball::getVelAngle()
{
    return this->vel_angle_;
}

void Ball::setX(float x)
{
    this->x_ = x;
}

void Ball::setY(float y)
{
    this->y_ = y;
}

void Ball::setVelX(float vel_x)
{
    this->vel_x_ = vel_x;
}

void Ball::setVelY(float vel_y)
{
    this->vel_y_ = vel_y;
}

void Ball::setVelAbs(float vel_abs)
{
    this->vel_abs_ = vel_abs;
}

void Ball::setVelAngle(float vel_angle)
{
    this->vel_angle_ = vel_angle;
}

/**
 * Update all internal state variables of the ball, such as velocities, based on its new (X, Y) coordinates.
 * Angular velocity is set in rad/s.
 * @param x x coordinate, in centimeters.
 * @param y y coordinate, in centimeters.
 */
void Ball::refreshState(float x, float y)
{
    ROS_DEBUG("Refreshing ball state");
    
    // calculate velocities
    this->setVelX(x - this->getX());
    this->setVelY(y - this->getY());
    this->setVelAbs(sqrt(pow(this->getVelX(), 2) + pow(this->getVelY(), 2)));
    this->setVelAngle(atan2(y - this->getY(), x - this->getX()));
    
    // refresh position
    this->setX(x);
    this->setY(y);
    
    ROS_DEBUG("Ball X = %f", this->getX());
    ROS_DEBUG("Ball Y = %f", this->getY());
    ROS_DEBUG("Ball Vel X = %f", this->getVelX());
    ROS_DEBUG("Ball Vel Y = %f", this->getVelY());
    ROS_DEBUG("Ball Vel Abs= %f", this->getVelAbs());
    ROS_INFO("Ball Vel Angle = %f", this->getVelAngle()*180.0/M_PI); // printing in deg/s
}
