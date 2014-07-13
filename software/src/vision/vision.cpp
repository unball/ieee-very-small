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
 * Implements vision, which consists in six parts:
 * - Homography
 * - Pre-processing
 * - Segmentation
 * - Tracking
 * - Communication
 * - GUI
 */

#include <unball/vision/vision.hpp>
#include <ros/ros.h>

float Vision::getRobotLocation(int robot_number)
{
    ROS_DEBUG("Get robot %d location", robot_number);
    return robot_location_[robot_number];
}

void Vision::setRGBFrame(cv::Mat rgb_frame)
{
    checkFrameSize(rgb_frame);
    rgb_frame_ = rgb_frame;
}

void Vision::setDepthFrame(cv::Mat depth_frame)
{
    checkFrameSize(depth_frame);
    depth_frame_ = depth_frame;
}

/**
 * Check whether a frame has a valid size, i.e., neither the widht nor the height can be zero.
 */
void Vision::checkFrameSize(cv::Mat frame)
{
    if (frame.rows == 0 || frame.cols == 0)
    {
        ROS_ERROR("Invalid image frame received");
        exit(1);
    }
}

void Vision::run()
{
    ROS_DEBUG("Run vision");

    gui_.show(rgb_frame_);
}