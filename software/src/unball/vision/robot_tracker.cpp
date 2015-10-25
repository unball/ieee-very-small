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

    cv::findContours(depth_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i)
    {
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        if (boundingRect.area() > min_area_ and boundingRect.area() < max_area_)
        {
            RobotData robot_data = robot_identifier_.identifyRobot(rgb_frame, contours[i], boundingRect);
            int team = robot_data.team, id = robot_data.id;
            if (team == RobotData::ALLY)
                robots_[team][id].setPosition(robot_data);
            else
            {
                int index = getClosestOpponentRobot(robot_data.center_position);
                if (index != -1)
                    robots_[team][index].setPosition(robot_data);
            }
        }
    }
}

void RobotTracker::trackStep2(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{

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
