/**
 * @file   robot.cpp
 * @author Matheus Vieira Portela
 * @date   27/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class
 *
 * Implements robots for strategy
 */

#include "robot.hpp"
#include <cmath>
#include <ros/ros.h>

Robot::Robot()
{
    this->x_ = 0;
    this->y_ = 0;
    this->th_ = 0;
    this->lin_vel_ = 0;
    this->ang_vel_ = 0;
    this->motion_state_ = STOP;
    this->previous_motion_state_ = UNDEFINED;
}

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

float Robot::getAngVel()
{
    return this->ang_vel_;
}

MotionState Robot::getMotionState()
{
    return this->motion_state_;
}

MotionState Robot::getPreviousMotionState()
{
    return this->previous_motion_state_;
}

void Robot::setPosition(float x, float y)
{
    this->x_ = x;
    this->y_ = y;
}

void Robot::setTh(float th)
{
    this->th_ = th;
}

void Robot::setPose(float x, float y, float th)
{
    this->setPosition(x, y);
    this->setTh(th);
}

/**
 * Set linear velocity, saturated at the ROBOT_SATURATION_LIN_VEL level (arbitrary value).
 * @param lin_vel Desired linear velocity level.
 */
void Robot::setLinVel(float lin_vel)
{
    lin_vel = this->saturate(lin_vel, ROBOT_SATURATION_LIN_VEL);
    this->lin_vel_ = lin_vel;
}

/**
 * Set angular velocity, saturated at the ROBOT_SATURATION_ANG_VEL level (arbitrary value).
 * @param ang_vel Desired angular velocity level.
 */
void Robot::setAngVel(float ang_vel)
{
    ang_vel = this->saturate(ang_vel, ROBOT_SATURATION_ANG_VEL);
    this->ang_vel_ = ang_vel;
}

void Robot::setMotionState(MotionState motion_state)
{
    this->motion_state_ = motion_state;
}

void Robot::setPreviousMotionState(MotionState previous_motion_state)
{
    this->previous_motion_state_ = previous_motion_state;
}

/**
 * Return whether the robot has changed its motion state, configuring that the last motion has finished.
 * @return whether the motion has changed.
 */
bool Robot::hasMotionStateChanged()
{
    if (this->previous_motion_state_ == UNDEFINED)
        return false;
    
    return (this->previous_motion_state_ != this->motion_state_);
}

/**
 * Saturate a number x, forcing it to the interval (-limit) <= x <= (limit).
 * @param x The number to saturate.
 * @param limit The limit of the saturation interval.
 * @return The saturated number.
 */
float Robot::saturate(float x, float limit)
{
    if (x > limit)
        x = limit;
    else if (x < -limit)
        x = -limit;
    
    return x;
}

/**
 * Reduce an angle to the interval (-M_PI) <= angle < (M_PI).
 * @param angle The angle to saturate.
 * @return The reduced angle.
 */
float Robot::reduceAngle(float angle)
{
    while (angle < -M_PI)
        angle += 2*M_PI;
    while (angle >= M_PI)
        angle -= 2*M_PI;
        
    return angle;
}

/**
 * Calculate the distance from the robot's current position to a (x, y) coordinate by using the Pythagorean theorem.
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @return The calculated distance.
 */
float Robot::calculateDistance(float x, float y)
{
    float dx = this->getX() - x;
    float dy = this->getY() - y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

/**
 * Calculate the angle from the robot's current position to a (x, y) coordinate by using arc-tangent.
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @return The calculated angle.
 */
float Robot::calculateAngle(float x, float y)
{
    float dx = this->getX() - x;
    float dy = this->getY() - y;
    return atan2(dy, dx);
}
