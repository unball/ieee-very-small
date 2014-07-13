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

#include <unball/vision/vision.hpp>

#include <ros/ros.h>

Vision::Vision()
{
    has_field_center_ = false;
    robot_coordinates_.resize(6);
}

/**
 * Segments the depth and rgb images, and finds all the necessary data
 * from the state of the game.
 */
void Vision::run()
{
    ROS_DEBUG("Run vision");
    
    if (depth_frame_.image.rows == 0 && depth_frame_.image.cols == 0)
    {
        ROS_WARN("No image on camera_frame_. Nothing to do here.");
        return;
    }
    
    if (!has_field_center_) findFieldCenter();
    
    // Still to be implemented
    segmentDepth();
    segmentImage();
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
            rgb_frame_.header = camera_frame.header;
            rgb_frame_.encoding = camera_frame.encoding;
            camera_frame.image.copyTo(rgb_frame_.image);
            break;
        case Vision::DEPTH_IMAGE:
            depth_frame_.header = camera_frame.header;
            depth_frame_.encoding = camera_frame.encoding;
            camera_frame.image.copyTo(depth_frame_.image);
            break;
        default:
            ROS_ERROR("Invalid image type");
            break;
    } 
}

float Vision::getRobotLocation(int robot_number)
{
    ROS_DEBUG("Get robot %d location", robot_number);
    return robot_location_[robot_number];
}

/**
 * Finds the pixel corresponding to the center of the field
 * TODO(gabri.navess@gmail.com): Implement this method properly.
 */
void Vision::findFieldCenter()
{
    ROS_DEBUG("Finding the field center");
    
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

