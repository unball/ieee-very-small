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

std::vector<cv::Point2f> GUI::rgb_points_;

GUI::GUI()
{
    rgb_frame_title_ = "RGB frame";
    depth_frame_title_ = "Depth frame";
    cv::namedWindow(rgb_frame_title_);
    cv::namedWindow(depth_frame_title_);
    cv::setMouseCallback(rgb_frame_title_, rgbMouseCallback, NULL);
}

void GUI::setRGBFrame(cv::Mat rgb_frame)
{
    // Check frame size
    if (rgb_frame.cols == 0 or rgb_frame.rows == 0)
    {
        ROS_WARN("Cannot set RGB frame of size %d x %d", rgb_frame.cols, rgb_frame.rows);
        return;
    }

    rgb_frame_ = rgb_frame;
}

void GUI::setDepthFrame(cv::Mat depth_frame)
{
    // Check frame size
    if (depth_frame.cols == 0 or depth_frame.rows == 0)
    {
        ROS_WARN("Cannot set depth frame of size %d x %d", depth_frame.cols, depth_frame.rows);
        return;
    }

    depth_frame_ = depth_frame;
}

/**
 * Shows an image on the screen
 * @param image OpenCV image that will be presented on the screen
 */
void GUI::show(cv::Mat image)
{
    cv::imshow("Vision GUI", image);
    cv::waitKey(1); // Must be called to show images sequentially
}

void GUI::showRGBFrame()
{
    cv::imshow(rgb_frame_title_, rgb_frame_);
    cv::waitKey(1); // Must be called to show images sequentially
}

void GUI::showDepthFrame()
{
    cv::Mat normalized_depth_frame;

    /*
     * The depth image is received in 16UC_1 format. This means values can go from 0 to 65,535 representing the distance
     * in milimeters. However, rarely distances will be greater than 2,5 meters in a robot soccer match. This means that
     * all pixels will be black when showing the image.
     * To make the image visible, we apply a normalization from 0 to 65,535.
     */
    cv::normalize(depth_frame_, normalized_depth_frame, 0, 65535, cv::NORM_MINMAX, CV_16UC1);

    cv::imshow(depth_frame_title_, normalized_depth_frame);
    cv::waitKey(1); // Must be called to show images sequentially
}

void GUI::rgbMouseCallback(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        ROS_INFO("RGB frame button click at: (%d,%d)", x, y);
        rgb_points_.push_back(cv::Point2f(x, y));
    }
}

std::vector<cv::Point2f> GUI::getRGBPoints()
{
    return rgb_points_;
}