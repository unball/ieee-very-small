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

TrackedRobot::TrackedRobot()
{
    type_ = ROBOT;
}

TrackedRobot::~TrackedRobot()
{
}

void TrackedRobot::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{
}

void TrackedRobot::updatePosition(cv::Point position)
{
    position_.x = exponentialMovingAvg(position_.x, position.x);
    position_.y = exponentialMovingAvg(position_.y, position.y);
}

void TrackedRobot::setPosition(cv::Point position)
{
	position_ = position;
}

int TrackedRobot::exponentialMovingAvg(int old_value, int new_value)
{
    return (0.25*new_value + (1.0-0.25)*old_value);
}

void TrackedRobot::draw(cv::Mat &frame)
{
	cv::circle(frame, position_, 10, (255, 0, 255));
}