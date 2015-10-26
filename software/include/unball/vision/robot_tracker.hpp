/**
 * @file   robot_tracker.hpp
 * @author Gabriel Naves da Silva
 * @date   24/07/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot tracker definition file
 */

#ifndef UNBALL_VISION_ROBOT_TRACKER_H_
#define UNBALL_VISION_ROBOT_TRACKER_H_

#include <vector>

#include <ros/ros.h>

#include <unball/vision/robot_identifier.hpp>
#include <unball/vision/tracked_robot.hpp>

class RobotTracker
{
  public:
    RobotTracker(MeasurementConversion *measurement_conversion_);
    void loadConfig();

    void track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &depth_segmented_frame);
    void draw(cv::Mat &frame);

    std::vector<float> getRobotPose(int robot_index);

  private:
    void trackStep1(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);
    void trackStep2(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame);
    int getClosestOpponentRobot(cv::Point new_position);
    float distanceBetweenPoints(cv::Point a, cv::Point b);
    void setNewRobot(RobotData robot_data);
    bool foundAllRobots();

    RobotIdentifier robot_identifier_;

    int robot_amount_;
    TrackedRobot robots_[2][3]; // First line for allied robots, second for opponent robots

    int tracking_step_;

    // Used on tracking step 1
    int min_area_, max_area_;

    // used for tracking system to make sure no more than 3 opponent robots are identified
    bool used_opponent_robots_[3];
    bool used_allied_robots_[3];

    // used to identify whether the tracking situation is stable
    int continuous_frame_counter_;
};

#endif // UNBALL_VISION_ROBOT_TRACKER_H_
