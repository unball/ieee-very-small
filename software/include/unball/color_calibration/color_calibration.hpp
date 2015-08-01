/**
 * @file   color_calibration.hpp
 * @author Gabriel Naves da Silva
 * @date   01/08/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Color calibration
 *
 * Defines the computer vision color calibration class.
 */

#ifndef UNBALL_COLOR_CALIBRATION_H_
#define UNBALL_COLOR_CALIBRATION_H_

#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class ColorCalibration
{
  public:
    ColorCalibration();
    static ColorCalibration& getInstance();

    void init(std::string color_name);
    void run();
    void setImage(cv::Mat rgb_image);

  private:
    void makeResultingImage();

    static ColorCalibration* instance;

    cv::Mat rgb_image_;
    cv::Mat resulting_image_;

    std::string color_name_;

    std::string rgb_window_name_;
    std::string resulting_window_name_;
    std::string trackbar_window_name_;

    int hsv_min_h_, hsv_min_s_, hsv_min_v_;
    int hsv_max_h_, hsv_max_s_, hsv_max_v_;

    bool has_image_;
};

#endif // UNBALL_COLOR_CALIBRATION_H_

