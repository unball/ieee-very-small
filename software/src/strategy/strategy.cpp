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
    ROS_INFO("Run strategy");
}

void Strategy::setRobotLocation(float location, int robot_number)
{
    ROS_INFO("Set robot %d location", robot_number);
    this->robots_location_[robot_number] = location;
}

float Strategy::getRobotVelocity(int robot_number)
{
    ROS_INFO("Get robot %d velocity", robot_number);
    return this->robots_velocities_[robot_number];
}
