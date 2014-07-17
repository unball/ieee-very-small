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

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

class Preprocessor
{
  public:
    Preprocessor();
    ~Preprocessor();
    cv::Mat preprocess(cv::Mat image);
    void preprocessDepth(cv::Mat image);

  private:
    void printMeanMinMax(cv::Mat image);
    std::string window_name_;
};

#endif // UNBALL_VISION_PREPROCESSOR_H_
