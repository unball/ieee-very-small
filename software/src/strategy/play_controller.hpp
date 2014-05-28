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
#include "play_stop.hpp"

enum PlayId
{
    NO_PLAY = -1,
    PLAY_STOP,
    PLAY_1,
    PLAY_2,
};

class PlayController
{
  public:
    PlayController();
    void run();
    void setPlay(PlayId play_number);
    void pushPlay(PlayId play_number);
    void clearPlayQueue();
    void abortPlay();
    
  private:
    void updatePlay();
    void executePlay();
    void mutexLock();
    void mutexUnlock();
    bool isMutexUnlocked();
    
    bool play_mutex_;
    std::queue<PlayId> play_queue_;
    PlayId current_play_;
    Play1 play1_;
    PlayStop play_stop_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

