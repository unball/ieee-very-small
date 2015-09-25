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
    void loadShowImage();
    void loadHSVMinSConfig();
    void loadHSVMinVConfig();
    void loadHSVAdjustConfig();
    void loadDepthSegmentationConfig();
    cv::Mat segment(cv::Mat image);
    cv::Mat segmentDepth(cv::Mat image);

  private:
    void findConnectedComponentsInDepthImage(cv::Mat image,
        std::map<int, std::pair<int, std::stack<cv::Point> > > &object_map, int &biggest_object_id);
    void fillDepthMaskImage(cv::Mat &mask,
        std::map<int, std::pair<int, std::stack<cv::Point> > > &object_map, int biggest_object_id);
    void depthSegAnalyzePixel(cv::Point original_pixel, cv::Point pixel_to_analyze, cv::Mat image,
        std::map<int, std::pair<int, std::stack<cv::Point> > > &object_map);

    std::string window_name_;
    bool show_image_;
    int hsv_min_s_;
    int hsv_min_v_;

    // Depth segmentation stuff
    std::queue<cv::Point> unknown_pixels_, current_object_;
    cv::Mat processed_points_;
    std::string depth_window_name_;
    int depth_threshold_;
    bool depth_seg_use_8_neighbours_;
    bool show_depth_image_;
};

#endif // UNBALL_VISION_SEGMENTER_H_
