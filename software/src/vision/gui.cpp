/**
 * @file   gui.cpp
 * @author Matheus Vieira Portela
 * @date   12/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of GUI for the vision module
 */

#include <unball/vision/gui.hpp>
#include <ros/ros.h>

GUI::GUI()
{
    window_name_ = "Vision GUI";
    cv::namedWindow(window_name_);
}

GUI::~GUI()
{
    cv::destroyWindow(window_name_);
}

/**
 * Shows an image on the screen
 * @param image OpenCV image that will be presented on the screen
 */
void GUI::show(cv::Mat image)
{
    if (image.cols == 0 || image.rows == 0)
    {
        ROS_WARN("Cannot show image of size %d x %d", image.cols, image.rows);
        return;
    }

    cv::imshow(window_name_, image);
    cv::waitKey(1); // 
}