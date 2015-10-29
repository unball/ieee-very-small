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
#include <map>
#include <stack>
#include <queue>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class Segmenter
{
  public:
    Segmenter();
    ~Segmenter();
    void loadConfig();
    cv::Mat segmentRGB(cv::Mat image);
    cv::Mat segmentDepth(cv::Mat image);

    void setFieldMask(cv::Mat field_mask);

  private:
    void loadShowImage();
    void loadHSVMinHConfig();
    void loadHSVMaxHConfig();
    void loadHSVMinSConfig();
    void loadHSVMinVConfig();
    void loadHSVAdjustConfig();
    void loadDepthSegmentationConfig();

    void removeExteriorOfField(cv::Mat &image);

    std::string window_name_;
    bool show_image_;
    int hsv_min_s_;
    int hsv_min_v_;
    int hsv_min_h_;
    int hsv_max_h_;

    // Depth segmentation stuff
    std::string depth_window_name_;
    int depth_threshold_;
    int depth_threshold_divider_;
    int size_value_;
    int depth_morphology_amount_;
    bool show_depth_image_;
    bool depth_adjust_;

    cv::Mat field_mask_;
    int outside_val_;
};

#endif // UNBALL_VISION_SEGMENTER_H_
