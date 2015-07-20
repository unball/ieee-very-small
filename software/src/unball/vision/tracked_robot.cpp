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

void TrackedRobot::loadConfig()
{
    int n_robots;
    ros::param::get("/vision/tracker/robots_per_team", n_robots);
    ros::param::get("/vision/tracker/identify_robots", identify_robots_);
    ros::param::get("/vision/segmenter/hsv_min_s", hsv_min_s_);
    ros::param::get("/vision/segmenter/hsv_min_v", hsv_min_v_);
    allied_robots_.resize(n_robots);
    opponent_robots_.resize(n_robots);
}

void TrackedRobot::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{
    std::vector< std::vector<cv::Point> > contours;

    cv::findContours(rgb_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i)
        identifyRobot(rgb_frame, contours[i], i);

    for (int i = 0; i < allied_robots_.size(); ++i)
        cv::circle(rgb_frame, allied_robots_[i], 20, cv::Scalar(180, 0, 180), 2);
    for (int i = 0; i < opponent_robots_.size(); ++i)
        cv::circle(rgb_frame, opponent_robots_[i], 20, cv::Scalar(0, 0, 180), 2);
}

void TrackedRobot::identifyRobot(cv::Mat &rgb_frame, std::vector<cv::Point> contour, int index)
{
    cv::Rect tracking_window = cv::boundingRect(contour);
    cv::Point center = calculateCenterPosition(tracking_window);

    if (not identify_robots_)
    {
        if (index == 0)
            allied_robots_[0] = center;
        else
            opponent_robots_[0] = center;
        return;
    }

    // Gets the point in contour that is farthest from the center
    cv::Point farthest_point = contour[0];
    float distance = distanceBetweenPoints(center, farthest_point);
    for (int k = 1; k < contour.size(); ++k)
    {
        float current_distance = distanceBetweenPoints(center, contour[k]);
        if (current_distance > distance)
        {
            farthest_point = contour[k];
            distance = current_distance;
        }
    }

    cv::Point opposite_point = calculateOppositePoint(farthest_point, center);
    cv::Point middle_f = calculateMidPoint(farthest_point, center);
    cv::Point middle_o = calculateMidPoint(opposite_point, center);

    cv::Mat hsv;
    cv::cvtColor(rgb_frame, hsv, CV_BGR2HSV);
    cv::Vec3b value_f = hsv.at<cv::Vec3b>(middle_f.y,middle_f.x);
    cv::Vec3b value_o = hsv.at<cv::Vec3b>(middle_o.y,middle_o.x);

    if (isPointRed(value_f) or isPointRed(value_o))
        opponent_robots_[0] = center;
    else if (isPointPink(value_f) or isPointPink(value_o))
        allied_robots_[0] = center;
    else
        ROS_ERROR("[TrackedRobot]identifyRobot: could not identify the robots team");
}

cv::Point TrackedRobot::calculateCenterPosition(cv::Rect tracking_window)
{
    cv::Point center;
    cv::Point top_left = tracking_window.tl();
    cv::Point bottom_right = tracking_window.br();
    center.x = (top_left.x + bottom_right.x)/2;
    center.y = (top_left.y + bottom_right.y)/2;
    return center;
}

double TrackedRobot::distanceBetweenPoints(cv::Point a, cv::Point b)
{
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2));
}

cv::Point TrackedRobot::calculateOppositePoint(cv::Point point, cv::Point reference)
{
    return cv::Point(2 * reference.x - point.x, 2 * reference.y - point.y);
}

cv::Point TrackedRobot::calculateMidPoint(cv::Point point, cv::Point reference)
{
    return cv::Point(reference.x + (point.x-reference.x)/2, reference.y + (point.y-reference.y)/2);
}

bool TrackedRobot::isPointRed(cv::Vec3b hsv_values)
{
    if (hsv_values[2] < hsv_min_v_ or hsv_values[1] < hsv_min_s_ or hsv_values[0] < 170)
        return false;
    return true;
}

bool TrackedRobot::isPointPink(cv::Vec3b hsv_values)
{
    if (hsv_values[2] < hsv_min_v_ or hsv_values[1] < hsv_min_s_ or hsv_values[0] > 175 or hsv_values[1] > 200)
        return false;
    return true;
}
