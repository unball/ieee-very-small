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
    this->last_motion_state_ = UNDEFINED;
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

/**
 * Set linear velocity, saturated at 5 (arbitrary value).
 * @param lin_vel Desired linear velocity level.
 */
void Robot::setLinVel(float lin_vel)
{
    lin_vel = this->saturate(lin_vel, ROBOT_SATURATION_LIN_VEL);
    this->lin_vel_ = lin_vel;
}

/**
 * Set angular velocity, saturated at 2 (arbitrary value).
 * @param ang_vel Desired angular velocity level.
 */
void Robot::setAngVel(float ang_vel)
{
    ang_vel = this->saturate(ang_vel, ROBOT_SATURATION_ANG_VEL);
    this->ang_vel_ = ang_vel;
}

void Robot::setMotionState(MotionState motion_state)
{
    ROS_DEBUG("Changing state from %d to %d", this->motion_state_, motion_state);
    this->motion_state_ = motion_state;
}

/**
 * Saturate a number x, forcing it to the interval (-limit) <= x <= (limit).
 * @param x The number to saturate.
 * @param limit The limit of the saturation interval.
 */
float Robot::saturate(float x, float limit)
{
    if (x > limit)
        x = limit;
    else if (x < -limit)
        x = -limit;
    
    return x;
}

bool Robot::hasMotionStateChanged()
{
    return (this->last_motion_state_ != this->motion_state_);
}

/**
 * Execute an action with respect to the current locomotion state.
 */
void Robot::run()
{
    ROS_DEBUG("Robot state: %d", this->getMotionState());
    
    this->last_motion_state_ = this->motion_state_;
    
    switch (this->getMotionState())
    {
        case STOP:
            ROS_DEBUG("STOP");
            this->executeStop();
            break;
        case MOVE:
            ROS_DEBUG("MOVE");
            this->executeMove();
            break;
        case TURN:
            ROS_DEBUG("TURN");
            this->executeTurn();
            break;
    }
}

/**
 * Change robot locomotion state to STOP, if it is not yet in it.
 */
void Robot::stop()
{
    if (this->getMotionState() != STOP)
        this->setMotionState(STOP);
}

/**
 * Stop all robot movements.
 */
void Robot::executeStop()
{
    ROS_DEBUG("Robot stopped");
    
    this->setLinVel(0);
    this->setAngVel(0);
}

/**
 * Move a specified distance. Change robot locomotion state to MOVE, if it is not yet in it, and initialize movement
 * variables.
 * @param distance Desired distance for the robot to move.
 */
void Robot::move(float distance)
{
    if (this->getMotionState() != MOVE)
    {
        ROS_INFO("Robot move: %f", distance);
        
        this->move_distance_ = distance;
        this->move_initial_x_ = this->getX();
        this->move_initial_y_ = this->getY();
        this->setMotionState(MOVE);
    }
}

/**
 * Calculates the travelled distance from the initial move position to the current position. Keeps moving while the
 * travelled distance is less than the allowed tolerance.
 */
void Robot::executeMove()
{
    ROS_DEBUG("Robot moving");
    
    const float tolerance = 0.02; // 2 cm
    float vel;
    float dx, dy, travelled_distance;
    
    dx = this->getX() - this->move_initial_x_;
    dy = this->getY() - this->move_initial_y_;
    
    travelled_distance = sqrt(pow(dx, 2) + pow(dy, 2));
    
    if (this->move_distance_ >= 0)
        vel = 1;
    else
        vel = -1;
    
    ROS_INFO("dx: %f", dx);
    ROS_INFO("dy: %f", dy);
    ROS_INFO("move distance: %f", this->move_distance_);
    ROS_INFO("travelled distance: %f", travelled_distance);
    ROS_INFO("difference: %f", (this->move_distance_ - travelled_distance));
    ROS_INFO("tolerance: %f", tolerance);
    ROS_INFO("abs: %f", fabs(this->move_distance_) - travelled_distance);
    
    if ((fabs(this->move_distance_) - travelled_distance) > tolerance)
    {
        ROS_INFO("Still moving");
        this->setLinVel(vel);
    }
    else
    {
        ROS_INFO("Finished moving");
        this->setMotionState(STOP);
    }
}

/**
 * Turn a specified angle. Change robot locomotion state to TURN, if it is not yet in it, and initialize turn variables.
 * @param angle Desired angle for the robot to turn.
 */
void Robot::turn(float angle)
{
    if (this->getMotionState() != TURN)
    {
        ROS_DEBUG("Robot turn: %f", angle);
        
        this->turn_angle_ = angle;
        this->turn_initial_th_ = this->getTh();
        this->setMotionState(TURN);
    }
}

/**
 * Calculates the turned angle from the initial orientation to the current orientation. Keeps moving while the turned
 * angle is less than the allowed tolerance.
 */
void Robot::executeTurn()
{
    ROS_DEBUG("Robot turning");
    
    const float tolerance = 0.1; // 1 rad
    float dth;
    
    dth = this->getTh() - this->turn_initial_th_;
    
    if ((this->turn_angle_ - dth) > tolerance)
    {
        ROS_DEBUG("Still turning");
        this->setAngVel(1);
    }
    else
    {
        ROS_DEBUG("Finished turning");
        this->setMotionState(STOP);
    }
}
