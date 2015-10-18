/**
 * @file   preprocessor.hpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the preprocessor class
 */

#ifndef UNBALL_VISION_PREPROCESSOR_H_
#define UNBALL_VISION_PREPROCESSOR_H_

#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class Preprocessor
{
  public:
    Preprocessor();
    ~Preprocessor();
    void loadConfig();
    void preprocessRGB(cv::Mat &rgb_frame);
    void preprocessDepth(cv::Mat &depth_frame);
    void preprocess(cv::Mat &rgb_frame, cv::Mat &depth_frame);

  private:
    void printMeanMinMax(const cv::Mat &image);
    bool show_image_;
    std::string window_name_;
};

#endif // UNBALL_VISION_PREPROCESSOR_H_
