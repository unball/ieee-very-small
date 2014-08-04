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
#include <ros/ros.h>
#include <unball/strategy/action_controller.hpp> // ActionControler action_controller

PlayController::PlayController()
{
    setPlay(NULL); // Don't have any initial play
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
 * @param *play new play to set.
 */
void PlayController::setPlay(Play *play)
{
    if (play == NULL) ROS_INFO("Set play: NONE");
    // Dont set if that is already the actual play. shouldnt be a problem, since we are controlling this with a mutex
    else ROS_INFO("Set play: %s", play->getPlayName().c_str());
    delete play_;
    play_ = play;
}

/**
 * Push a new play to the play queue.
 * @param play new play that will be pushed.
 */
void PlayController::pushPlay(Play *play)
{
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
    ROS_INFO("Abort play");
    
    setPlay(new PlayStop());
    clearPlayQueue();
}

/**
 * Change the current play ID number, which occures whenever the play mutex is unlocked.
 */
void PlayController::updatePlay()
{
    if (play_ != NULL)
        ROS_INFO("UPDATE PLAY: %s", play_->getPlayName().c_str());
    else
        ROS_INFO("UPDATE PLAY: NONE");

    if (isMutexUnlocked())
    {
        if (not play_queue_.empty())
        {
            setPlay(play_queue_.front());
            play_queue_.pop();
        }
        else
        {
            setPlay(NULL);
        }

        mutexLock();
    }
}

/*
void PlayController::selectPlay()
{
    std::string play_name = (play_==NULL) ? "NO PLAY" : play_->getPlayName();
    if (play_name == "NO PLAY")
        has_play_finished = true; // Always unlock for the next play
    else if (play_name == "PLAY STOP")
        has_play_finished = play_stop_.run();
    else if (play_name == "PLAY 1")
        has_play_finished = play1_.run();
    else if (play_name == "PLAY FORMATION 1")
        has_play_finished = play_formation_1_.run();
    else if (play_name == "PLAY FORMATION 2")
        has_play_finished = play_formation_2_.run();
    else
        ROS_ERROR("Unknown play ID number");
}
*/

/**
 * Execute a play based on the current play ID number.
 */
void PlayController::executePlay()
{
    bool has_play_finished;
    
    has_play_finished = (play_ == NULL) ? true : play_->run();
    
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
