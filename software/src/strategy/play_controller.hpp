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

#include <vector>
#include <queue>
#include "action_controller.hpp"
#include "robot.hpp" // robot[6]
#include "ball.hpp"
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
    void play1(); // TODO: make it a specific class that inherits from Play.
    
  private:
    void mutexLock();
    void mutexUnlock();
    bool isMutexUnlocked();
  
    ActionController action_controller_; // TODO: remove when play1 is a class
    bool robots_action_finished_[6]; // TODO: remove when play1 is a class
    int play_state_; // TODO: remove when play1 is a class
    bool play_mutex_; // TODO: move to strategy class
    std::queue<int> play_set_;
    int current_play_;
    Play1 play1_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

