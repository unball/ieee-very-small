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
#include "robot.hpp"
#include "ball.hpp"

class Strategy
{
  public:
    Strategy();
  
    void run();
    
    // Robot methods
    void setRobotPose(int robot_number, float x, float y, float th);
    std::vector<float> getRobotVelocities(int robot_number);
    void runRobots();
    
    // Ball methods
    void setBallLocation(float x, float y);
    
    // Action methods
    void updateAction();
    void executeAction();
    void actionExample(int robot_number);
    void actionLookAndGo(int robot_number);
    
  private:
    Robot robots_[6];
    
    Ball ball_;
    
    int action_state_;
    bool action_mutex_;
    std::queue<int> action_set_;
    int current_action_;
};

#endif  // UNBALL_STRATEGY_H_
