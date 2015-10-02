/**
 * @file   robot_tracker.cpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot tracker implementation file
 */

#include <unball/vision/robot_tracker.hpp>

RobotTracker::RobotTracker()
{
    tracking_step_ = 1;
}

void RobotTracker::loadConfig()
{
    ros::param::get("/vision/tracker/robots_per_team", robot_amount_);
    robot_identifier_.loadConfig();
}

void RobotTracker::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{
    switch (tracking_step_)
    {
        case 1:
            trackStep1(rgb_frame, depth_frame, rgb_segmented_frame);
            break;
        case 2:
            trackStep2(rgb_frame, depth_frame, rgb_segmented_frame);
            break;
        default:
            ROS_ERROR("[RobotTracker]track: unknown tracking step");
            break;
    }
}

void RobotTracker::draw(cv::Mat &frame)
{
    // for (int i = 0; i < 2; ++i)
    //     for (int j = 0; j < robot_amount_; ++j)
    //         robots_[i][j].draw(frame);
}

void RobotTracker::trackStep1(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &depth_segmented_frame)
{
    std::vector< std::vector<cv::Point> > contours;

    cv::findContours(depth_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i)
    {
        ROS_ERROR("sizeof Contours[%d]: %lu", i, contours[i].size());
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        cv::rectangle(rgb_frame, boundingRect, cv::Scalar(0, 255, 0));
        // RobotData robot_data = robot_identifier_.identifyRobot(rgb_frame, contours[i]);
        // robots_[robot_data.team][robot_data.id].setPosition(robot_data);
    }
}

void RobotTracker::trackStep2(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{

}

std::vector<float> RobotTracker::getRobotPose(int robot_index)
{
    return robots_[robot_index / 3][robot_index % 3].getRobotPose();
}
