/**
 * @file   play_controller.hpp
 * @author Matheus Vieira Portela
 * @date   25/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 *
 * Plays are sequences of asynchronous actions for multiple robots.
 * PlayController controls sequences of plays that will be executed by the strategy module.
 */

#ifndef UNBALL_PLAY_CONTROLLER_H_
#define UNBALL_PLAY_CONTROLLER_H_

#include <queue>

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/no_play.hpp>
#include <unball/strategy/play_stop.hpp>
#include <unball/strategy/play_formation_1.hpp>
#include <unball/strategy/play_formation_2.hpp>

class PlayController
{
  public:
    PlayController();
    void run();
    void setPlay(Play *play);
    void pushPlay(Play *play);
    void clearPlayQueue();
    void abortPlay();
    
  private:
    void updatePlay();
    void executePlay();
    void mutexLock();
    void mutexUnlock();
    bool isMutexUnlocked();
    
    bool play_mutex_;
    std::queue<Play*> play_queue_;
    
    Play *play_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

