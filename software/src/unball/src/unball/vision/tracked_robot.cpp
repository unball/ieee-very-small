/**
 * @file   tracked_robot.cpp
 * @author Matheus Vieira Portela
 * @date   05/08/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class implementation for tracker
 */

#include <unball/vision/tracked_robot.hpp>

// Exponential moving average constant
const float TrackedRobot::AVG_CONSTANT = 0.25;

// Bounding rectangle color in BGR
const cv::Scalar TrackedRobot::TEAM_1_COLOR(0, 0, 255);

// Bounding rectangle color in BGR
const cv::Scalar TrackedRobot::TEAM_2_COLOR(255, 0, 255);

TrackedRobot::TrackedRobot(cv::Mat frame)
{
    frame.copyTo(frame_);
}

/**
 * Calculates exponential moving average. This filter updates the value by giving more importance to new values than
 * older ones according to a constant value.
 * The average constant indicates the level of confidence in the measured values, from 0 to 1. Higher values indicate
 * that new measurements are relatively precise and should receive more importance than older ones.
 * @param old_value Previous accumulated value
 * @param new_value Measured value
 * @return Averaged value according to the constant
 */
int TrackedRobot::exponentialMovingAvg(int old_value, int new_value)
{
    return (AVG_CONSTANT*new_value + (1.0-AVG_CONSTANT)*old_value);
}

/**
 * Updates the field position using an exponential moving average.
 * @param position measured position
 */
void TrackedRobot::updatePosition(cv::Point position)
{
    position_.x = exponentialMovingAvg(position_.x, position.x);
    position_.y = exponentialMovingAvg(position_.y, position.y);
}

/**
 * Updates the field bounding rectangle using an exponential moving average.
 * @param bounding_rect new bounding rectangle
 */
void TrackedRobot::updateBoundingRect(cv::Rect bounding_rect)
{
    bounding_rect_.x = exponentialMovingAvg(bounding_rect_.x, bounding_rect.x);
    bounding_rect_.y = exponentialMovingAvg(bounding_rect_.y, bounding_rect.y);
    bounding_rect_.width = exponentialMovingAvg(bounding_rect_.width, bounding_rect.width);
    bounding_rect_.height = exponentialMovingAvg(bounding_rect_.height, bounding_rect.height);
}

/**
 * Draw bounding rectangle and central circle
 * @param frame OpenCV image frame to draw on
 */
void TrackedRobot::drawMarker(cv::Mat &frame)
{
    cv::rectangle(frame, bounding_rect_, TEAM_1_COLOR);
}