/**
 * @file   play_controller.cpp
 * @author Matheus Vieira Portela
 * @date   25/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play controller
 *
 * Implements strategy for robots
 */

#include "play_controller.hpp"
#include <ros/ros.h>
#include "action_controller.hpp" // ActionControler action_controller
#include "robot.hpp" // Robot robot[6];
#include "ball.hpp" // Ball ball;

PlayController::PlayController()
{
    this->current_play_ = NO_PLAY;
    this->mutexUnlock(); // Allow to change plays
    
    // Example play set
    this->play_set_.push(1);
    this->play_set_.push(0); // Empty play
}

void PlayController::run()
{
    ROS_INFO("Run play controller");
    
    this->updatePlay();
    this->executePlay();
    action_controller.run();
}

/**
 * Change the current play ID number, which occures whenever the play mutex is unlocked.
 */
void PlayController::updatePlay()
{
    ROS_INFO("UPDATE PLAY: %d", this->current_play_);
    
    if (this->isMutexUnlocked())
    {
        if (not this->play_set_.empty())
        {
            this->current_play_ = this->play_set_.front();
            this->play_set_.pop();
        }
        else
        {
            this->current_play_ = NO_PLAY;
        }
        
        this->mutexLock();
    }
}

/**
 * Execute a play based on the current play ID number.
 */
void PlayController::executePlay()
{
    switch (this->current_play_)
    {
        case 1:
            if (this->play1_.run())
                this->mutexUnlock();
            break;
        default: // No play
            break;
    }
}

void PlayController::mutexLock()
{
    this->play_mutex_ = false;
}

void PlayController::mutexUnlock()
{
    this->play_mutex_ = true;
}

bool PlayController::isMutexUnlocked()
{
    return this->play_mutex_;
}
