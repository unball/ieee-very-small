/**
 * @file   tracked_object.cpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracked object implementation file for vision module
 */

#include <unball/vision/tracked_object.hpp>

TrackedObject::TrackedObject()
{
    type_ = NONE;
    orientation_ = 0.0;
}

TrackedObject::~TrackedObject()
{
}

/**
 * Initialize variables
 * @param tracking_window OpenCV rectangle containing the object
 */
void TrackedObject::init(cv::Rect &tracking_window)
{
    tracking_window_ = tracking_window;
}

void TrackedObject::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{
}

void TrackedObject::draw(cv::Mat &frame)
{
    cv::rectangle(frame, tracking_window_, cv::Scalar(255, 0, 255), 2);
}
