/**
 * @file   segmenter.hpp
 * @author Matheus Vieira Portela
 * @date   12/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the segmenter class
 */

#ifndef UNBALL_VISION_SEGMENTER_H_
#define UNBALL_VISION_SEGMENTER_H_

#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class Segmenter
{
  public:
    Segmenter();
    ~Segmenter();
    void loadConfig();
    void loadHSVAdjustConfig();
    void setNodeHandle(ros::NodeHandle *n);
    cv::Mat segment(cv::Mat image);

  private:
    ros::NodeHandle *n_;

    std::string window_name_;
    int s_min_;
    int v_min_;
};

#endif // UNBALL_VISION_SEGMENTER_H_