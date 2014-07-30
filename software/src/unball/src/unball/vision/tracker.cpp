/**
 * @file   tracker.cpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracker implementation file for vision module
 */

#include <unball/vision/tracker.hpp>

Tracker::Tracker()
{
    window_name_ = "Tracker";
}

Tracker::~Tracker()
{
    if (show_image_)
        cv::destroyWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Tracker::loadConfig()
{
    ros::param::get("/vision/tracker/show_image", show_image_);
    ROS_INFO("Tracker show image: %d", show_image_);

    if (show_image_)
        cv::namedWindow(window_name_);
}

/**
 * @param image segmented OpenCV image
 */
void Tracker::track(cv::Mat preprocessed, cv::Mat segmented)
{
    cv::Mat tracked;
    std::vector<cv::Mat> bgr_channels;

    // Apply the segmented image mask to the processed image channel by channel
    cv::split(preprocessed, bgr_channels);
    for (int i = 0; i < bgr_channels.size(); ++i)
        bgr_channels[i] &= segmented;
    cv::merge(bgr_channels, tracked);

    if (show_image_)
    {
        cv::imshow(window_name_, tracked);
        cv::waitKey(1);
    }
}