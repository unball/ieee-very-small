/**
 * @file   vision.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
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
    
    if (camera_frame_.image.rows == 0 && camera_frame_.image.cols == 0)
    {
        ROS_WARN("No image on camera_frame_. Nothing to do here.");
        return;
    }
}

void Vision::setCameraFrame(cv_bridge::CvImage camera_frame)
{
    ROS_DEBUG("Set camera frame");
    this->camera_frame_.header = camera_frame.header;
    this->camera_frame_.encoding = camera_frame.encoding;
    camera_frame.image.copyTo(this->camera_frame_.image);
}

float Vision::getRobotLocation(int robot_number)
{
    ROS_DEBUG("Get robot %d location", robot_number);
    return this->robot_location_[robot_number];
}
