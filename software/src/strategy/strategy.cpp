/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Strategy class
 *
 * Defines strategy for robots
 */

#include "strategy.hpp"
#include <ros/ros.h>

Strategy::Strategy()
{
    // Init robots positions according to the simulator positions
    float x[6] = {0.37, 0.37, 0.60, -0.37, -0.37, -0.60};
    float y[6] = {0.40, -0.40, 0.0, 0.40, -0.40, 0.0};
    
    for (int i = 0; i < 6; ++i)
    {
        this->setRobotPose(i, x[i], y[i], 0.0);
        this->robots_[i].stop();
    }
    
    this->action_set_.push(1);
    //this->action_set_.push(2);
    this->action_set_.push(0);
    
    this->action_state_ = 0;
    this->action_mutex_ = true;
    this->current_action_ = -1;
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    ROS_INFO("Run strategy");
    
    this->updateAction();
    this->executeAction();
    this->runRobots();
}

void Strategy::setRobotPose(int robot_number, float x, float y, float th)
{
    this->robots_[robot_number].setX(x);
    this->robots_[robot_number].setY(y);
    this->robots_[robot_number].setTh(th);
}

std::vector<float> Strategy::getRobotVelocities(int robot_number)
{
    float lin_vel = this->robots_[robot_number].getLinVel();
    float ang_vel = this->robots_[robot_number].getAngVel();
    std::vector<float> velocities;
    
    velocities.push_back(lin_vel);
    velocities.push_back(ang_vel);
    
    return velocities;
}

/**
 * Call for each robot's run method.
 */
void Strategy::runRobots()
{
    for (int i = 0; i < 6; ++i)
        this->robots_[i].run();
}

void Strategy::setBallLocation(float x, float y)
{
    this->ball_.updatePosition(x, y);
}

/**
 * Executes an example play. First, it moves behind and, then, forward. Changes state each time the robot
 * also changes its motion state (configuring that the current motion has stopped).
 */
void Strategy::actionExample(int robot_number)
{
    if (this->robots_[robot_number].hasMotionStateChanged())
        ++this->action_state_;
    
    switch (this->action_state_)
    {
        case 0:
            ROS_INFO("ACTION EXAMPLE STATE 0");
            this->robots_[robot_number].goTo(0.0, 0.15);
            break;
        case 1:
            ROS_INFO("ACTION EXAMPLE STATE 1");
            this->robots_[robot_number].lookAt(0.0, 0.0);
            break;
        case 2:
            ROS_INFO("ACTION EXAMPLE STATE 2");
            this->robots_[robot_number].move(0.15);
            break;
        default:
            ROS_INFO("ACTION EXAMPLE FINISHED");
            this->action_mutex_ = true;
            break;
    }
}

void Strategy::actionLookAndGo(int robot_number)
{
    float dx, dy, distance;
    
    if (this->robots_[robot_number].hasMotionStateChanged())
        ++this->action_state_;
    
    switch (this->action_state_)
    {
        case 0:
            ROS_INFO("ACTION LOOK AND GO STATE 0");
            this->robots_[robot_number].lookAt(0.0, 0.0);
            break;
        case 1:
            ROS_INFO("ACTION LOOK AND GO STATE 1");
            dx = this->robots_[robot_number].getX() - 0.0;
            dy = this->robots_[robot_number].getY() - 0.0;
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            this->robots_[robot_number].move(distance);
            break;
        default:
            ROS_INFO("ACTION LOOK AND GO FINISHED");
            this->action_mutex_ = true;
            break;
    }
}

void Strategy::updateAction()
{
    ROS_INFO("UPDATE ACTION: %d", this->current_action_);
    
    if (this->action_mutex_)
    {
        if (not this->action_set_.empty())
        {
            this->current_action_ = this->action_set_.front();
            this->action_set_.pop();
        }
        else
        {
            this->current_action_ = 0;
        }
        
        this->action_state_ = 0;
        this->action_mutex_ = false;
    }
}

void Strategy::executeAction()
{
    switch (this->current_action_)
    {
        case 1:
            this->actionExample(3);
            break;
        case 2:
            this->actionLookAndGo(3);
            break;
        default: // No action
            break;
    }
}
