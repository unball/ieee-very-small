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
#include <unball/strategy/play_stop.hpp>
#include <unball/strategy/play1.hpp>
#include <unball/strategy/play_formation_1.hpp>
#include <unball/strategy/play_formation_2.hpp>

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

