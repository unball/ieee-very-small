/**
 * @file   robot.cpp
 * @author Matheus Vieira Portela
 * @date   27/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Robot class
 *
 * Implements robots for strategy
 */

#include "robot.hpp"

float Robot::getX()
{
    return this->x_;
}

float Robot::getY()
{
    return this->y_;
}

float Robot::getTh()
{
    return this->th_;
}

float Robot::getLinVel()
{
    return this->lin_vel_;
}

float Robot::getRotVel()
{
    return this->rot_vel_;
}

void Robot::setX(float x)
{
    this->x_ = x;
}

void Robot::setY(float y)
{
    this->y_ = y;
}

void Robot::setTh(float th)
{
    this->th_ = th;
}

void Robot::setLinVel(float lin_vel)
{
    this->lin_vel_ = lin_vel;
}

void Robot::setRotVel(float rot_vel)
{
    this->rot_vel_ = rot_vel;
}
