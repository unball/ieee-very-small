/**
 * @file   tracker.hpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracker definition file for vision module
 */

#ifndef UNBALL_VISION_TRACKER_H_
#define UNBALL_VISION_TRACKER_H_

#include <string>
#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <unball/vision/tracked_field.hpp>
#include <unball/vision/tracked_robot.hpp>

#define BAD_CONFIG 1

class Tracker
{
  public:
    Tracker();
    ~Tracker();
    void loadShowImage();
    void loadFieldTrackingMode();
    void loadConfig();
    void trackRobots(cv::Mat rgb_frame, cv::Mat depth_frame, cv::Mat rgb_segmented_frame);
    void track(cv::Mat rgb_frame, cv::Mat depth_frame, cv::Mat rgb_segmented_frame);

  private:
    std::string window_name_;
    bool show_image_;
    TrackedField tracked_field_;
    TrackedRobot tracked_robot_;
};

#endif // UNBALL_VISION_TRACKER_H_