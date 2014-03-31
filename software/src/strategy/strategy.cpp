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

void Strategy::run()
{
    ROS_DEBUG("Run strategy");
}

void Strategy::setRobotLocation(float x, float y, int robot_number)
{
    ROS_DEBUG("Set robot %d location", robot_number);
    this->robots_[robot_number].setX(x);
    this->robots_[robot_number].setY(y);
}

std::vector<float> Strategy::getRobotVelocities(int robot_number)
{
    ROS_DEBUG("Get robot %d velocities", robot_number);
    float lin_vel = this->robots_[robot_number].getLinVel();
    float rot_vel = this->robots_[robot_number].getRotVel();
    std::vector<float> velocities;
    velocities.push_back(lin_vel);
    velocities.push_back(rot_vel);
    return velocities;
}
