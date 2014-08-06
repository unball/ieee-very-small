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

#include <unball/vision/tracked_object.hpp>

class TrackedRobot : public TrackedObject
{
  public:
    TrackedRobot(cv::Mat frame);
    int exponentialMovingAvg(int old_value, int new_value);
    void updatePosition(cv::Point position);
    void updateBoundingRect(cv::Rect bounding_rect);
    void drawMarker(cv::Mat &frame);

  private:
    static const float AVG_CONSTANT;
    static const cv::Scalar TEAM_1_COLOR;
    static const cv::Scalar TEAM_2_COLOR;

    cv::Mat frame_;
};

#endif // UNBALL_VISION_TRACKED_ROBOT_H_