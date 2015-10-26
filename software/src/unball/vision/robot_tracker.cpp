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

RobotTracker::RobotTracker(MeasurementConversion *mc)
{
    tracking_step_ = 1;
    continuous_frame_counter_ = 0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            robots_[i][j].setMeasurementConversion(mc);
}

void RobotTracker::loadConfig()
{
    ros::param::get("/vision/tracker/robots_per_team", robot_amount_);
    robot_identifier_.loadConfig();
    min_area_ = 800;
    max_area_ = 1730;
    // cv::namedWindow("Robot tracker config");
    // cv::createTrackbar("Min area", "Robot tracker config", &min_area_, 20000);
    // cv::createTrackbar("Max area", "Robot tracker config", &max_area_, 20000);
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
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            robots_[i][j].draw(frame);
}

void RobotTracker::trackStep1(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &depth_segmented_frame)
{
    std::vector< std::vector<cv::Point> > contours;
    memset(used_opponent_robots_, false, sizeof(used_opponent_robots_));
    memset(used_allied_robots_, false, sizeof(used_allied_robots_));

    cv::findContours(depth_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i)
    {
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        if (boundingRect.area() > min_area_ and boundingRect.area() < max_area_)
        {
            RobotData robot_data = robot_identifier_.identifyRobot(rgb_frame, contours[i], boundingRect);
            setNewRobot(robot_data);
        }
    }

    if (foundAllRobots())
        continuous_frame_counter_++;
    else
        restartRobotFilters();

    if (continuous_frame_counter_ >= 20)
        tracking_step_ = 2;
}

void RobotTracker::setNewRobot(RobotData robot_data)
{
    int team = robot_data.team, id = robot_data.id;
    if (team == RobotData::ALLY)
    {
        robots_[team][id].setPosition(robot_data);
        used_allied_robots_[id] = true;
    }
    else
    {
        int index = getClosestOpponentRobot(robot_data.center_position);
        if (index != -1)
            robots_[team][index].setPosition(robot_data);
    }
}

bool RobotTracker::foundAllRobots()
{
    for (int i = 0; i < 3; ++i)
        if (used_allied_robots_[i] == false || used_opponent_robots_[i] == false)
            return false;
    return true;
}

void RobotTracker::restartRobotFilters()
{
    continuous_frame_counter_ = 0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            robots_[i][j].filter_.restart();
}

void RobotTracker::trackStep2(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &depth_segmented_frame)
{
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            trackIndividualRobot(rgb_frame, depth_segmented_frame, robots_[i][j]);
}

void RobotTracker::trackIndividualRobot(cv::Mat &rgb_frame, cv::Mat &depth_segmented_frame, TrackedRobot &robot)
{
    robot.filter_.predict();
    cv::Point2f predicted_position = robot.filter_.getPredictedPose();
    float window_width = 45, window_height = 45;
    cv::Point2f upper_left_corner(predicted_position.x-(window_width/2.0), predicted_position.y-(window_height/2.0));
    cv::Rect prediction_window(upper_left_corner.x, upper_left_corner.y, window_width, window_height);
    cv::Mat roi = depth_segmented_frame(prediction_window);

    cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));
    cv::morphologyEx(roi, roi, cv::MORPH_ERODE, structuring_element, cv::Point(-1,-1), 8);

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size() == 1)
    {
        cv::RotatedRect robot_outline = cv::minAreaRect(contours[0]);
        robot_outline.center += cv::Point2f(upper_left_corner.x, upper_left_corner.y);
        robot_outline.size += cv::Size2f(10,10);
        float orientation = robot_outline.angle*2*M_PI/360.0;
        chooseCorrectOrientation(orientation, robot);
        robot.setPosition(robot_outline.center, robot_outline, orientation);
    }
    else
    {
        robot.setPosition(predicted_position);
    }
}

void RobotTracker::chooseCorrectOrientation(float &orientation, TrackedRobot &robot)
{
    float closest_orientation = orientation;
    float previous_orientation = robot.getOrientation();
    for (int i = 1; i <= 4; ++i)
    {
        float current_orientation = orientation+M_PI*i/2;
        if (abs(current_orientation - previous_orientation) < abs(closest_orientation - previous_orientation))
            closest_orientation = current_orientation;
    }
    orientation = closest_orientation;
}

int RobotTracker::getClosestOpponentRobot(cv::Point new_position)
{
    int result = -1;
    float closest_distance = 1000000000;
    for (int i = 0; i < 3; ++i)
    {
        float current_distance = distanceBetweenPoints(new_position, robots_[1][i].getPixelPosition());;
        if (current_distance < closest_distance && used_opponent_robots_[i] == false)
        {
            closest_distance = current_distance;
            result = i;
        }
    }
    if (result != -1)
        used_opponent_robots_[result] = true;
    return result;
}

float RobotTracker::distanceBetweenPoints(cv::Point a, cv::Point b)
{
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2));
}

std::vector<float> RobotTracker::getRobotPose(int robot_index)
{
    return robots_[robot_index / 3][robot_index % 3].getRobotPose();
}
