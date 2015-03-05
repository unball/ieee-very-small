/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the homography class
 */

#ifndef UNBALL_VISION_HOMOGRAPHY_H_
#define UNBALL_VISION_HOMOGRAPHY_H_

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class Homography
{
  public:
    Homography();
    void calcHomographyMat(std::vector<cv::Point2f> src_points);
    cv::Mat transform(cv::Mat image);

    bool isHomographyDone();

  private:
    bool is_done_;

    std::vector<cv::Point2f> dst_points_;
    
    cv::Mat homography_matrix_;
};

#endif // UNBALL_VISION_HOMOGRAPHY_H_