/**
 * @file   gui.hpp
 * @author Matheus Vieira Portela
 * @date   12/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of GUI for the vision module
 *
 * This GUI class deals with the following tasks:
 * - Show images in proper windows
 * - Handle mouse input for calibration and training
 * - Draw circles in the robots positions
 */

#ifndef UNBALL_VISION_GUI_H_
#define UNBALL_VISION_GUI_H_

#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class GUI
{
  public:
    void setRGBFrame(cv::Mat rgb_frame);
    void setDepthFrame(cv::Mat depth_frame);
    void show(cv::Mat image);
    void showRGBFrame();
    void showDepthFrame();

  private:
    cv::Mat rgb_frame_;
    cv::Mat depth_frame_;
};

#endif // UNBALL_VISION_GUI_H_
