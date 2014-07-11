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
#include "play_stop.hpp"
#include "play1.hpp"
#include "play_formation_1.hpp"
#include "play_formation_2.hpp"

enum PlayId
{
    NO_PLAY = -1,
    PLAY_STOP,
    PLAY_1,
    PLAY_FORMATION_1,
    PLAY_FORMATION_2,
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
    PlayStop play_stop_;
    Play1 play1_;
    PlayFormation1 play_formation_1_;
    PlayFormation2 play_formation_2_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

