/**
 * @file   measurement_conversion.hpp
 * @author Gabriel Naves da Silva
 * @date   29/06/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Class responsible for converting the measurements from pixel values to the metric system.
 *
 * Defines the MesurementConversion class.
 */

#ifndef UNBALL_VISION_MEASUREMENT_CONVERSION_H_
#define UNBALL_VISION_MEASUREMENT_CONVERSION_H_

#include <iostream>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class MeasurementConversion
{
  public:
    MeasurementConversion();
    void loadConfig();
    void calculateConversion(float field_pixel_width, float field_pixel_height);
    cv::Point convertToMetric(cv::Point point_in_pixel);
    cv::Point convertToPixel(cv::Point point_in_metric);

  private:
    bool has_calculated_parameters_;
    float field_metric_width_, field_metric_height_;
    std::pair<float, float> conversion_parameters_; // first is for x axis, second is for y axis
};

#endif // UNBALL_VISION_MEASUREMENT_CONVERSION_H_
