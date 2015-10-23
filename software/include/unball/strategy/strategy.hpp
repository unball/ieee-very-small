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
 * Defines strategy for robots. Strategy is a singleton.
 */

#ifndef UNBALL_STRATEGY_H_
#define UNBALL_STRATEGY_H_

#include <ros/ros.h>

#include <unball/strategy/state_estimator.hpp>
#include <unball/strategy/trajectory_controller.hpp>

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
    static Strategy& getInstance();
    Strategy();
    void receiveKeyboardInput(char key);
    void run();
    
  private:
    static Strategy *instance; // singleton instance

    StateEstimator state_estimator_;
    TrajectoryController trajectory_controller_;

    void PauseGame();
    void ResumeGame();

    void updatePlayers();
};

extern Strategy strategy;

#endif  // UNBALL_STRATEGY_H_
