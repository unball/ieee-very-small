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

/**
 * Segments the depth and rgb images, and finds all the necessary data
 * from the state of the game.
 */
void Vision::run()
{
    ROS_DEBUG("Run vision");
    
    if (rgb_frame_.image.rows == 0 && rgb_frame_.image.cols == 0)
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

/**
 * Sets both the rgb frame and the depth frame.
 * @param camera_frame The frame to be set.
 * @param image_type Identifier for the image given,
 * either rgb or depth.
 */
void Vision::setCameraFrame(cv_bridge::CvImage camera_frame, int image_type)
{
    ROS_DEBUG("Set camera frame");
    switch(image_type)
    {
        case Vision::RGB_IMAGE:
            this->rgb_frame_.header = camera_frame.header;
            this->rgb_frame_.encoding = camera_frame.encoding;
            camera_frame.image.copyTo(this->rgb_frame_.image);
            break;
        case Vision::DEPTH_IMAGE:
            this->depth_frame_.header = camera_frame.header;
            this->depth_frame_.encoding = camera_frame.encoding;
            camera_frame.image.copyTo(this->depth_frame_.image);
            break;
        default:
            ROS_ERROR("Invalid image type");
            break;
    } 
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
    
    if (rgb_frame_.image.rows == 0 && rgb_frame_.image.cols == 0)
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
