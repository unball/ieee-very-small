/**
 * @file   play_controller.hpp
 * @author Matheus Vieira Portela
 * @date   25/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 *
 * Defines strategy plays
 */

#ifndef UNBALL_PLAY_CONTROLLER_H_
#define UNBALL_PLAY_CONTROLLER_H_

#include <queue>
#include "play1.hpp"

#define NO_PLAY 0
#define INITIAL_PLAY_STATE 0

class PlayController
{
  public:
    PlayController();
  
    void run();
    void updatePlay();
    void executePlay();
    
  private:
    void mutexLock();
    void mutexUnlock();
    bool isMutexUnlocked();
    
    bool play_mutex_; // TODO: move to strategy class
    std::queue<int> play_set_;
    int current_play_;
    Play1 play1_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

