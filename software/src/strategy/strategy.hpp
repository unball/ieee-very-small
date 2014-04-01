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
#include "robot.hpp"
#include "ball.hpp"

class Strategy
{
  public:
    void run();
    
    void setRobotLocation(float x, float y, int robot_number);
    std::vector<float> getRobotVelocities(int robot_number);
    
    void setBallLocation(float x, float y);
    
  private:
    Robot robots_[6];
    Ball ball_;
};

#endif  // UNBALL_STRATEGY_H_
