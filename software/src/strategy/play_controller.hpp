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
    void play1();
    
  private:
    ActionController action_controller_;
    bool robots_action_finished_[6];
    int play_state_;
    bool play_mutex_;
    std::queue<int> play_set_;
    int current_play_;
};

#endif  // UNBALL_PLAY_CONTROLLER_H_

