/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the homography class.
 */

#ifndef UNBALL_VISION_HOMOGRAPHY_H_
#define UNBALL_VISION_HOMOGRAPHY_H_

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <unball/vision/gui.hpp>

class Homography
{
  public:
    enum HomographyStep
    {
        CALIBRATION,
        RECTIFICATION,
        END
    };

    Homography();

    void loadConfig();
    void run(std::vector<cv::Point2f> rgb_points, std::vector<cv::Point2f> depth_points);

    cv::Mat calibrate(cv::Mat image);
    cv::Mat rectify(cv::Mat image);

    bool isHomographyDone();
    bool isCalibrationDone();

  private:
    void calcCalibrationMat(std::vector<cv::Point2f> src_points, std::vector<cv::Point2f> dst_points);
    void calcHomographyMat(std::vector<cv::Point2f> src_points);

    HomographyStep current_step_;

    std::vector<cv::Point2f> dst_points_;
    
    cv::Mat calibration_matrix_;
    cv::Mat homography_matrix_;
};

#endif // UNBALL_VISION_HOMOGRAPHY_H_