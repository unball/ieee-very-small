/**
 * @file   homography.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the homography class.
 *
 * The homography algorithm is divided in three steps
 * - Calibration: The calibration matrix is calculated, so as to match depth with RGB images
 * - Rectification: The rectification matrix is calculated, so as to rectify the RGB and depth images
 * - End: The calculated matrices are used to calibrate depth images and rectify RGB and depth images as requested.
 */

#ifndef UNBALL_VISION_HOMOGRAPHY_H_
#define UNBALL_VISION_HOMOGRAPHY_H_

#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

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
    void saveMatrix(std::string file_name, cv::Mat &matrix);
    void loadMatrix(std::string file_name, cv::Mat &matrix);

    HomographyStep current_step_;

    std::vector<cv::Point2f> dst_points_; // Points for rectification
    std::string calib_matrix_file_name_; // Name for the file containing the calibration matrix
    std::string rect_matrix_file_name_; // Name for the file containing the rectification matrix
    
    cv::Mat calibration_matrix_;
    cv::Mat homography_matrix_;

    bool overwrite_calibration_matrix_;
    bool overwrite_rectification_matrix_;
};

#endif // UNBALL_VISION_HOMOGRAPHY_H_
