/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Strategy class
 *
 * Implements strategy for robots
 */

#include "strategy.hpp"
#include <ros/ros.h>

/**
 * Strategy constructor.
 */
Strategy::Strategy()
{
    this->play_set_.push(1);
    this->play_set_.push(2);
    this->play_set_.push(0);
    
    this->play_state_ = 0;
    this->play_mutex_ = true;
    this->current_play_ = -1;
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    ROS_INFO("Run strategy");
    
    this->updatePlay();
    this->executePlay();
    this->action_controller_.run();
}

/**
 * Executes an example play. First, it moves behind and, then, forward. Changes state each time the robot also changes
 * its motion state (configuring that the current motion has stopped).
 */
void Strategy::playExample(int robot_number)
{
    if (robot[robot_number].hasMotionStateChanged())
        ++this->play_state_;
    
    switch (this->play_state_)
    {
        case 0:
            ROS_INFO("PLAY EXAMPLE STATE 0");
            this->action_controller_.goTo(robot_number, -0.15, 0.15);
            break;
        case 1:
            ROS_INFO("PLAY EXAMPLE STATE 1");
            this->action_controller_.goTo(robot_number, -0.40, 0.2);
            break;
        default:
            ROS_INFO("PLAY EXAMPLE FINISHED");
            this->play_mutex_ = true;
            break;
    }
}

/**
 * Executes a look and go play. First, it looks at (0, 0) and then it moves to (0, 0). Changes state each time the robot
 * also changes its motion state (configuring that the current motion has stopped).
 */
void Strategy::playLookAndGo(int robot_number)
{
    float dx, dy, distance;
    
    if (robot[robot_number].hasMotionStateChanged())
        ++this->play_state_;
    
    switch (this->play_state_)
    {
        case 0:
            ROS_INFO("PLAY LOOK AND GO STATE 0");
            this->action_controller_.lookAt(robot_number, 0.0, 0.0);
            break;
        case 1:
            ROS_INFO("PLAY LOOK AND GO STATE 1");
            dx = robot[robot_number].getX() - 0.0;
            dy = robot[robot_number].getY() - 0.0;
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            this->action_controller_.move(robot_number, distance);
            break;
        default:
            ROS_INFO("PLAY LOOK AND GO FINISHED");
            this->play_mutex_ = true;
            break;
    }
}

/**
 * Change the current play ID number, which occures whenever the play mutex is set to true.
 */
void Strategy::updatePlay()
{
    ROS_INFO("UPDATE PLAY: %d", this->current_play_);
    
    if (this->play_mutex_)
    {
        if (not this->play_set_.empty())
        {
            this->current_play_ = this->play_set_.front();
            this->play_set_.pop();
        }
        else
        {
            this->current_play_ = 0;
        }
        
        this->play_state_ = 0;
        this->play_mutex_ = false;
    }
}

/**
 * Execute a play based on the current play ID number.
 */
void Strategy::executePlay()
{
    switch (this->current_play_)
    {
        case 1:
            this->playExample(3);
            break;
        case 2:
            this->playLookAndGo(3);
            break;
        default: // No play
            break;
    }
}
