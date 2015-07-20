/**
 * @file   tracked_robot.hpp
 * @author Matheus Vieira Portela
 * @date   31/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class definition for tracker
 */

#ifndef UNBALL_VISION_TRACKED_ROBOT_H_
#define UNBALL_VISION_TRACKED_ROBOT_H_

#include <ros/ros.h>

#include <unball/vision/tracked_object.hpp>

class TrackedRobot : public TrackedObject
{
  public:
    TrackedRobot();
    ~TrackedRobot();
    void loadConfig();
    void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);

  private:
    void identifyRobot(cv::Mat &rgb_frame, std::vector<cv::Point> contour, int index);
    cv::Point calculateCenterPosition(cv::Rect tracking_window);
    double distanceBetweenPoints(cv::Point a, cv::Point b);
    cv::Point calculateOppositePoint(cv::Point point, cv::Point reference);
    cv::Point calculateMidPoint(cv::Point point, cv::Point reference);
    bool isPointRed(cv::Vec3b hsv_values);
    bool isPointPink(cv::Vec3b hsv_values);

    std::vector<cv::Point> allied_robots_;
    std::vector<cv::Point> opponent_robots_;
    bool identify_robots_;
    int hsv_min_s_;
    int hsv_min_v_;
};

#endif // UNBALL_VISION_TRACKED_ROBOT_H_
