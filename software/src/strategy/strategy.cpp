/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Strategy class
 *
 * Implements strategy for robots
 */

#include "strategy.hpp"
#include <ros/ros.h>

/**
 * Strategy global object
 */
Strategy strategy;

Strategy::Strategy()
{
    this->play_controller_.pushPlay(PLAY_1);
    this->game_state_ = GAME_RUNNING;
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    if (not this->game_state_ == GAME_PAUSED)
    {
        this->choosePlay();
        this->play_controller_.run();
    }
}

void Strategy::choosePlay()
{
    
}
