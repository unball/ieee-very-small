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
#include "robot.hpp"
#include "ball.hpp"

class PlayController
{
  public:
    PlayController();
  
    void run();
    void updatePlay();
    void executePlay();
    void play1(); //TODO: make it a specific class that inherits from Play.
    
  private:
    ActionController action_controller_; //remove when play1 is a class
    bool robots_action_finished_[6]; //remove when play1 is a class
    int play_state_; //remove when play1 is a class
    bool play_mutex_;
    std::queue<int> play_set_;
    int current_play_;
    void mutexLock();
    void mutexUnlock();
    bool mutexIsLocked();
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

