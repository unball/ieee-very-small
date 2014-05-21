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

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    ROS_INFO("Run strategy");
    
    this->play_controller_.run();
}
