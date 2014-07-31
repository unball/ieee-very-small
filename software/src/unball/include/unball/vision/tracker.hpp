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

#include <unball/vision/tracked_object.hpp>

class Tracker
{
  public:
    Tracker();
    ~Tracker();
    void loadConfig();
    void trackField(cv::Mat rgb_frame);
    void updateFieldCenter(cv::Point field_center);
    void track(cv::Mat preprocessed, cv::Mat segmented);

  private:
    std::string window_name_;
    bool show_image_;
    cv::Point field_center_;
};

#endif // UNBALL_VISION_TRACKER_H_