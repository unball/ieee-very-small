/**
 * @file   strategy.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Strategy class
 *
 * Defines strategy for robots
 */

#include "strategy.hpp"

#include <ros/ros.h>

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    ROS_DEBUG("Run strategy");
}

void Strategy::setRobotPose(int robot_number, float x, float y, float th)
{
    this->robots_[robot_number].setX(x);
    this->robots_[robot_number].setY(y);
    this->robots_[robot_number].setTh(th);
}

std::vector<float> Strategy::getRobotVelocities(int robot_number)
{
    float lin_vel = this->robots_[robot_number].getLinVel();
    float ang_vel = this->robots_[robot_number].getAngVel();
    std::vector<float> velocities;
    
    velocities.push_back(lin_vel);
    velocities.push_back(ang_vel);
    
    return velocities;
}

void Strategy::setBallLocation(float x, float y)
{
    this->ball_.updatePosition(x, y);
}
