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

#include <unball/strategy/play_controller.hpp>

PlayController::PlayController()
{
    setPlay(NO_PLAY); // Don't have any initial play
    mutexUnlock(); // Allow to change plays
}

void PlayController::run()
{
    ROS_INFO("Run play controller");
    
    updatePlay();
    executePlay();
    action_controller.run();
}

/**
 * Set the current play.
 * @param play_number Number of the play to set.
 */
void PlayController::setPlay(PlayId play_number)
{
    ROS_INFO("Set play: %d", play_number);
    
    current_play_ = play_number;
}

/**
 * Push a new play to the play queue.
 * @param play_number Number of the play that will be pushed.
 */
void PlayController::pushPlay(PlayId play_number)
{
    play_queue_.push(play_number);
}

/**
 * Clear the play queue. There is no method in the std::queue container for this, so we swap the current queue by an
 * empty version of the container.
 * Implementation suggested at: http://stackoverflow.com/questions/709146/how-do-i-clear-the-stdqueue-efficiently
 */
void PlayController::clearPlayQueue()
{
    std::queue<PlayId> empty_play_queue;
    std::swap(play_queue_, empty_play_queue);
}

/**
 * Interrupt the current play and clear the play queue.
 */
void PlayController::abortPlay()
{
    ROS_INFO("Abort play");
    
    setPlay(PLAY_STOP);
    clearPlayQueue();
}

/**
 * Change the current play ID number, which occures whenever the play mutex is unlocked.
 */
void PlayController::updatePlay()
{
    ROS_INFO("UPDATE PLAY: %d", current_play_);
    
    if (isMutexUnlocked())
    {
        if (not play_queue_.empty())
        {
            setPlay(play_queue_.front());
            play_queue_.pop();
        }
        else
        {
            setPlay(NO_PLAY);
        }
        
        mutexLock();
    }
}

/**
 * Execute a play based on the current play ID number.
 */
void PlayController::executePlay()
{
    bool has_play_finished;
    
    switch (current_play_)
    {
        case NO_PLAY:
            has_play_finished = true; // Always unlock for the next play
            break;
        case PLAY_STOP:
            has_play_finished = play_stop_.run();
            break;
        case PLAY_1:
            has_play_finished = play1_.run();
            break;
        case PLAY_FORMATION_1:
            has_play_finished = play_formation_1_.run();
            break;
        case PLAY_FORMATION_2:
            has_play_finished = play_formation_2_.run();
            break;
        default:
            ROS_ERROR("Unknown play ID number");
            break;
    }
    
    if (has_play_finished)
        mutexUnlock();
}

void PlayController::mutexLock()
{
    play_mutex_ = false;
}

void PlayController::mutexUnlock()
{
    play_mutex_ = true;
}

bool PlayController::isMutexUnlocked()
{
    return play_mutex_;
}
