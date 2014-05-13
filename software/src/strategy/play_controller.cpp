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

PlayController::PlayController()
{
    for (int i = 0; i < 6; ++i)
        this->robots_action_finished_[i] = false;
    
    this->play_state_ = 0; // No play state
    mutexUnlock(); // Allow to change plays
    this->current_play_ = -1; // No current play
    
    // Example play set
    this->play_set_.push(1);
    this->play_set_.push(0); // Empty play
}

void PlayController::run()
{
    ROS_INFO("Run play controller");
    
    this->updatePlay();
    this->executePlay();
    this->action_controller_.run();
}

/**
 * Change the current play ID number, which occures whenever the play mutex is unlocked.
 */
void PlayController::updatePlay()
{
    ROS_INFO("UPDATE PLAY: %d", this->current_play_);
    
    if (not mutexIsLocked())
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
        mutexLock();
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
            this->play1();
            break;
        default: // No play
            break;
    }
}

/**
 * Executes an example play, which moves robots 3 and 4. The play state should only be changed whenever all robots in
 * the play have finished their own actions.
 */
void PlayController::play1()
{
    for (int i = 0; i < 6; ++i)
    {
        if (robot[i].hasMotionStateChanged())
            this->robots_action_finished_[i] = true;
    }
    
    if (this->robots_action_finished_[3] && this->robots_action_finished_[4])
    {
        this->robots_action_finished_[3] = false;
        this->robots_action_finished_[4] = false;
        ++this->play_state_;
    }
    
    switch (this->play_state_)
    {
        case 0:
            ROS_INFO("PLAY EXAMPLE STATE 0");
            this->action_controller_.goTo(3, -0.15, 0.15);
            this->action_controller_.goTo(4, 0.30, 0.15);
            break;
        case 1:
            ROS_INFO("PLAY EXAMPLE STATE 1");
            this->action_controller_.goTo(3, -0.40, 0.2);
            this->action_controller_.goTo(4, -0.60, 0.2);
            break;
        default:
            ROS_INFO("PLAY EXAMPLE FINISHED");
            mutexUnlock();
            break;
    }
}

void PlayController::mutexLock()
{
    play_mutex_ = false;
}

void PlayController::mutexUnlock()
{
    play_mutex_ = true;
}

bool PlayController::mutexIsLocked()
{
    return(not play_mutex_);
}
