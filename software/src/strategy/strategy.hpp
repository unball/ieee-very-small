/**
 * @file   strategy.hpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Strategy class
 *
 * Defines strategy for robots
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

#include <vector>
#include <queue>
#include "action_controller.hpp"
#include "robot.hpp"
#include "ball.hpp"

class Strategy
{
  public:
    Strategy();
  
    void run();
    
    // Play methods
    void updatePlay();
    void executePlay();
    void playExample(int robot_number);
    void playLookAndGo(int robot_number);
    
  private:
    ActionController action_controller_;
    
    int play_state_;
    bool play_mutex_;
    std::queue<int> play_set_;
    int current_play_;
};

#endif  // UNBALL_STRATEGY_H_
