/**
 * @file   strategy.hpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Strategy class
 *
 * Defines strategy for robots
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

#include "play_controller.hpp"

class Strategy
{
  public:
    void run();
    
  private:
    PlayController play_controller_;
};

#endif  // UNBALL_STRATEGY_H_
