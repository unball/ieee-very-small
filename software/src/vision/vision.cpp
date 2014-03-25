/**
 * @file   vision.cpp
 * @author Matheus Vieira Portela
 * @date   23/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
 *
 * Implements computer vision algorithms
 */

#include "vision.hpp"

#include <ros/ros.h>

void Vision::run()
{
    ROS_DEBUG("Run vision");
}

void Vision::setCameraFrame(cv_bridge::CvImage camera_frame)
{
    ROS_DEBUG("Set camera frame");
    this->camera_frame_ = camera_frame;
}

float Vision::getRobotLocation(int robot_number)
{
    ROS_DEBUG("Get robot %d location", robot_number);
    return this->robot_location_[robot_number];
}
