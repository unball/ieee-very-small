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

#include <unball/vision/vision_polygon.hpp>
#include <unball/vision/gui.hpp>

class Preprocessor
{
  public:
    Preprocessor();
    ~Preprocessor();
    void loadConfig();
    void preprocessRGB(cv::Mat &rgb_frame);
    void preprocessDepth(cv::Mat &depth_frame);
    void preprocess(cv::Mat &rgb_frame, cv::Mat &depth_frame);

    void runFieldCalibration(std::vector<cv::Point2f> rgb_points);
    bool isFieldCalibrationDone();

  private:
    void printMeanMinMax(const cv::Mat &image);
    void fixDepthImageNoise(cv::Mat &image);
    void removeExteriorOfField(cv::Mat &image);

    void getMainPolygon(std::vector<cv::Point2f> rgb_points);
    void getGoalPolygon(std::vector<cv::Point2f> rgb_points);
    void calculateMask();

    bool show_depth_image_, adjust_noise_reduction_;
    int noise_thresh_;
    std::string window_name_;

    int outside_val_;
    bool has_main_field_;
    bool has_goal_;
    bool is_field_calibration_done_;
    VisionPolygon main_polygon_, goal_polygon_;
    cv::Mat field_mask_;
};

#endif // UNBALL_VISION_PREPROCESSOR_H_
