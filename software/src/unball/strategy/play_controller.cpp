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
    setPlay(new NoPlay()); // Don't have any initial play
    mutexUnlock(); // Allow to change plays
}

void PlayController::run()
{
    ROS_INFO("[PlayController] Run");
    
    updatePlay();
    executePlay();
    ActionController::getInstance().run();
}

/**
 * Set the current play.
 * @param *play new play to set.
 */
void PlayController::setPlay(Play *play)
{
    ROS_INFO("[PlayController] Set play: %s", play->getPlayName().c_str());
    delete play_;
    play_ = play;
}

/**
 * Push a new play to the play queue.
 * @param play new play that will be pushed.
 */
void PlayController::pushPlay(Play *play)
{
    ROS_INFO("[PlayController] Push play: %s", play_->getPlayName().c_str());
    play_queue_.push(play);
}

/**
 * Clear the play queue. There is no method in the std::queue container for this, so we swap the current queue by an
 * empty version of the container.
 * Implementation suggested at: http://stackoverflow.com/questions/709146/how-do-i-clear-the-stdqueue-efficiently
 */
void PlayController::clearPlayQueue()
{
    std::queue<Play*> empty_play_queue;
    std::swap(play_queue_, empty_play_queue);
}

/**
 * Interrupt the current play and clear the play queue.
 */
void PlayController::abortPlay()
{
    ROS_INFO("[PlayController] Abort play");
    
    setPlay(new PlayStop());
    clearPlayQueue();
}

/**
 * Change the current play ID number, which occures whenever the play mutex is unlocked.
 */
void PlayController::updatePlay()
{
    if (isMutexUnlocked())
    {
        ROS_INFO("[PlayController] Update play: %s", play_->getPlayName().c_str());

        if (not play_queue_.empty())
        {
            setPlay(play_queue_.front());
            play_queue_.pop();
        }
        else
        {
            ROS_INFO("[PlayController] Empty play queue");
            setPlay(new NoPlay());
        }

        mutexLock(); // Lock until the play finishes
    }
}

/**
 * Execute a play based on the current play ID number.
 */
void PlayController::executePlay()
{
    bool has_play_finished;

    ROS_INFO("[PlayController] Execute play: %s", play_->getPlayName().c_str());
    
    has_play_finished = play_->run();
    
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