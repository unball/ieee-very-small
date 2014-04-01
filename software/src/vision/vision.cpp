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

Vision::Vision()
{
    has_field_center_ = false;
}

void Vision::run()
{
    ROS_DEBUG("Run vision");
    
    if (camera_frame_.image.rows == 0 && camera_frame_.image.cols == 0)
    {
        ROS_WARN("No image on camera_frame_. Nothing to do here.");
        return;
    }
    
    segmentDepth();
    segmentImage();
    for (int i = 0; i < 6; i++)
    {
        findAngle(i);
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

/**
 * Finds the pixel corresponding to the center of the field
 */
void Vision::findFieldCenter()
{
    ROS_DEBUG("Find the field center");
    
    if (camera_frame_.image.rows == 0 && camera_frame_.image.cols == 0)
    {
        ROS_WARN("No image on camera_frame_. Nothing to do here.");
        return;
    }
    
    has_field_center_ = true;
}

/**
 * Segments the deph image and finds the
 * center pixel of all robots and the ball.
 */
void Vision::segmentDepth()
{
    ROS_DEBUG("Segmenting depth image");
}

/**
 * Segments the rgb image and finds the
 * front pixel of all robots.
 */
void Vision::segmentImage()
{
    ROS_DEBUG("Segmenting rgb image");
}

/**
 * Uses the information found on the segmentation
 * to determine the angle of a given robot.
 * @param robot_number the number of the robot.
 */
void Vision::findAngle(int robot_number)
{
    ROS_DEBUG("Find %d robot's angle", robot_number);
}
