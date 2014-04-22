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
    this->motion_state_ = motion_state;
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
 * Execute an action with respect to the current motion state. Whenever an execution returns true, meaning that it has
 * completed its goal, changes the motion state to STOP.
 */
void Robot::run()
{
    bool finished;
    
    ROS_DEBUG("Robot state: %d", this->getMotionState());
    
    this->previous_motion_state_ = this->motion_state_;
    
    switch (this->getMotionState())
    {
        case STOP:
            finished = this->executeStop();
            break;
        case MOVE:
            finished = this->executeMove();
            break;
        case LOOK_AT:
            finished = this->executeLookAt();
            break;
        case GO_TO:
            finished = this->executeGoTo();
            break;
    }
    
    if (finished)
        this->setMotionState(STOP);
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
 * @return always true, since it always is successful in stopping the movements.
 */
bool Robot::executeStop()
{
    ROS_DEBUG("Robot stopped");
    
    this->setLinVel(0);
    this->setAngVel(0);
    return true;
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
        ROS_DEBUG("Robot move: %f", distance);
        
        this->move_distance_ = distance;
        this->move_initial_x_ = this->getX();
        this->move_initial_y_ = this->getY();
        this->setMotionState(MOVE);
    }
}

/**
 * Calculates the travelled distance from the initial move position to the current position. Keeps moving while the
 * travelled distance is less than the allowed tolerance.
 * @return true if it has moved the required distance, false otherwise.
 */
bool Robot::executeMove()
{
    ROS_DEBUG("Robot moving");
    
    const float tolerance = 0.02; // 2 cm
    
    float dx = this->getX() - this->move_initial_x_;
    float dy = this->getY() - this->move_initial_y_;
    float travelled_distance = sqrt(pow(dx, 2) + pow(dy, 2));
    float error = fabs(this->move_distance_) - travelled_distance;
    float vel = (this->move_distance_ >= 0) ? 1 : -1;
    
    if (error > tolerance)
    {
        this->setLinVel(vel);
        return false;
    }
    else
    {
        this->setLinVel(0); // force it to a stop (may be unnecessary)
        return true;
    }
}

/**
 * Turn the robot to face a (x, y) coordinate. Change robot locomotion state to LOOK_AT, if it is not yet in it, and
 * initialize the look at variables.
 * @param x Desired x coordinate for the robot to turn.
 * @param y Desired y coordinate for the robot to turn.
 */
void Robot::lookAt(float x, float y)
{
    if (this->getMotionState() != LOOK_AT)
    {
        this->look_at_x_ = x;
        this->look_at_y_ = y;
        this->setMotionState(LOOK_AT);
    }
}

/**
 * Calculates the necessary angle to look at the (x, y) coordinate, based on the robot's current position. Keeps moving
 * while the difference between the current angle and the target angle is less than the allowed tolerance.
 * @return true if its has finished turning to the (x, y) coordinate, false otherwise.
 */
bool Robot::executeLookAt()
{
    const float tolerance = 0.1; // ~ 5.72 degrees
    
    float dx = this->getX() - this->look_at_x_;
    float dy = this->getY() - this->look_at_y_;
    float target_angle = atan2(dy, dx);
    float error = this->reduceAngle(this->getTh() - target_angle);
    float vel = (error > 0) ? 0.05 : -0.05;
    
    if (fabs(error) > tolerance)
    {
        this->setAngVel(vel);
        return false;
    }
    else
    {
        this->setAngVel(0);
        return true;
    }
}

/**
 * Move the robot to a (x, y) coordinate. Change robot locomotion state to GO_TO, if it is not yet in it, and initialize
 * the go to variables.
 * @param x Desired x coordinate for the robot to go.
 * @param y Desired y coordinate for the robot to go.
 */
void Robot::goTo(float x, float y)
{
    if (this->getMotionState() != GO_TO)
    {
        ROS_DEBUG("Robot go to: (%f, %f)", x, y);
        
        this->go_to_x_ = x;
        this->go_to_y_ = y;
        this->go_to_state_ = 0;
        this->setMotionState(GO_TO);
    }
}

/**
 * Makes the robot first look at the (x, y) coordinate, then, move the distance between the robot's current position and
 * the (x, y) point.
 * @return true if its has arrived at the (x, y) coordinate, false otherwise.
 */
bool Robot::executeGoTo()
{
    float dx, dy, distance;
    
    switch (this->go_to_state_)
    {
        case 0:
            // Look to the point
            this->look_at_x_ = this->go_to_x_;
            this->look_at_y_ = this->go_to_y_;
            if (this->executeLookAt())
                ++this->go_to_state_;
            break;
        case 1:
            // Move to the point
            dx = this->getX() - this->go_to_x_;
            dy = this->getY() - this->go_to_y_;
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            this->move_initial_x_ = this->getX();
            this->move_initial_y_ = this->getY();
            this->move_distance_ = distance;
            if (this->executeMove())
                ++this->go_to_state_;
            break;
        default:
            // Finished
            return true;
    }
    
    return false;
}
