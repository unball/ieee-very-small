/**
 * @file   segmenter.hpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   12/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the segmenter class
 */

#ifndef UNBALL_VISION_SEGMENTER_H_
#define UNBALL_VISION_SEGMENTER_H_

#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class Segmenter
{
  public:
    Segmenter();
    ~Segmenter();
    void loadConfig();
    void loadHSVMinSConfig();
    void loadHSVMinVConfig();
    void loadHSVAdjustConfig();
    cv::Mat segment(cv::Mat image);

  private:
    std::string window_name_;
    
    int hsv_min_s_;
    int hsv_min_v_;
};

#endif // UNBALL_VISION_SEGMENTER_H_
