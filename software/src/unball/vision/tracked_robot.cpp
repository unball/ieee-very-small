/**
 * @file   tracked_robot.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
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

void TrackedRobot::draw(cv::Mat &frame)
{
    cv::rectangle(frame, tracking_window_, team_color_, 2);
}

void TrackedRobot::setPosition(RobotData data)
{
    position_ = data.center_position;
    orientation_ = data.orientation;
    tracking_window_ = data.tracking_window;
    team_color_ = data.team_color;
}
