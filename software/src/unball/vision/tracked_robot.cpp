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
    std::vector< std::vector<cv::Point> > contours;
    cv::Rect tracking_window;
    cv::Point top_left;
    cv::Point bottom_right;
    cv::Point center;
    std::vector<cv::Mat> robots_frame;
    std::vector<cv::Point> robots_positions;

    // Find blobs from segmented image and extract images
    cv::findContours(rgb_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i)
    {
        // Find bounding rectangle
        tracking_window = cv::boundingRect(contours[i]);
        robots_frame.push_back(cv::Mat(rgb_frame(tracking_window)));

        // Calculate position
        top_left = tracking_window.tl();
        bottom_right = tracking_window.br();
        center.x = (top_left.x + bottom_right.x)/2;
        center.y = (top_left.y + bottom_right.y)/2;
        robots_positions.push_back(cv::Point(center));

        cv::rectangle(rgb_frame, tracking_window, (255, 0, 255), 2);
        // ROS_ERROR("Robot %d: (%d,%d)", i, center.x, center.y);
    }
}
