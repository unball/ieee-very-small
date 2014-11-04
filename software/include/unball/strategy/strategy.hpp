/**
 * @file   strategy.hpp
 * @author Matheus Vieira Portela
 * @author Icaro da Costa Mota
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

#include <ros/ros.h>

#include <unball/strategy/play_controller.hpp>
#include <unball/strategy/state_estimator.hpp>

namespace TeamState
{
    enum StrategyState
    {
        VERY_OFFENSIVE,
        OFFENSIVE,
        DEFENSIVE,
        STALLING
    };
};

class Strategy
{
  public:
    Strategy();
    void receiveKeyboardInput(char key);
    void run();
    
  private:
    void choosePlay();
    void chooseStrategyState();
    void findRobotClosestToBall();

    StateEstimator state_estimator_;
    PlayController play_controller_;
    TeamState::StrategyState strategy_state_;
};

extern Strategy strategy;

#endif  // UNBALL_STRATEGY_H_
