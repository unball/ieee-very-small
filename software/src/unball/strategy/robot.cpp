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

#include <unball/strategy/robot.hpp>

/**
 * Robot global object
 */
Robot robot[6];

Robot::Robot()
{
    pos_.set(0,0);
    th_ = 0;
    lin_vel_ = 0;
    ang_vel_ = 0;
    motion_state_ = STOP;
    previous_motion_state_ = UNDEFINED;
}

Point Robot::getPos()
{
    return(pos_);
}

float Robot::getX()
{
    return pos_.getX();
}

float Robot::getY()
{
    return pos_.getY();
}

float Robot::getTh()
{
    return th_;
}

float Robot::getLinVel()
{
    return lin_vel_;
}

float Robot::getAngVel()
{
    return ang_vel_;
}

MotionState Robot::getMotionState()
{
    return motion_state_;
}

MotionState Robot::getPreviousMotionState()
{
    return previous_motion_state_;
}

void Robot::setPosition(float x, float y)
{
    pos_.set(x,y);
}

void Robot::setTh(float th)
{
    th_ = th;
}

void Robot::setPose(float x, float y, float th)
{
    setPosition(x, y);
    setTh(th);
}

/**
 * Set linear velocity, saturated at the ROBOT_SATURATION_LIN_VEL level (arbitrary value).
 * @param lin_vel Desired linear velocity level.
 */
void Robot::setLinVel(float lin_vel)
{
    lin_vel = math::saturate(lin_vel, ROBOT_SATURATION_LIN_VEL);
    lin_vel_ = lin_vel;
}

/**
 * Set angular velocity, saturated at the ROBOT_SATURATION_ANG_VEL level (arbitrary value).
 * @param ang_vel Desired angular velocity level.
 */
void Robot::setAngVel(float ang_vel)
{
    ang_vel = math::saturate(ang_vel, ROBOT_SATURATION_ANG_VEL);
    ang_vel_ = ang_vel;
}

void Robot::setMotionState(MotionState motion_state)
{
    motion_state_ = motion_state;
}

void Robot::setPreviousMotionState(MotionState previous_motion_state)
{
    previous_motion_state_ = previous_motion_state;
}

/**
 * Return whether the robot has changed its motion state, configuring that the last motion has finished.
 * @return whether the motion has changed.
 */
bool Robot::hasMotionStateChanged()
{
    if (previous_motion_state_ == UNDEFINED)
        return false;
    
    return (previous_motion_state_ != motion_state_);
}