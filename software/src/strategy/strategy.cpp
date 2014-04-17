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

Strategy::Strategy()
{
    // Init robots positions according to the simulator positions
    float x[6] = {0.37, 0.37, 0.60, -0.37, -0.37, -0.60};
    float y[6] = {0.40, -0.40, 0.0, 0.40, -0.40, 0.0};
    
    for (int i = 0; i < 6; ++i)
    {
        this->setRobotPose(i, x[i], y[i], 0.0);
        //this->robots_[i].stop();
    }
}

/**
 * Run strategy methods that should be called each strategy iteration.
 */
void Strategy::run()
{
    ROS_DEBUG("Run strategy");
    
    this->robots_[0].move(-0.3);
    ROS_INFO("Robot 0: x: %f, y: %f, th: %f", this->robots_[0].getX(), this->robots_[0].getY(), this->robots_[0].getTh());
    this->runRobots();
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

void Strategy::runRobots()
{
    for (int i = 0; i < 6; ++i)
        this->robots_[i].run();
}

void Strategy::setBallLocation(float x, float y)
{
    this->ball_.updatePosition(x, y);
}
