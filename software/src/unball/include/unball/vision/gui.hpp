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
    GUI();
    ~GUI();
    void show(cv::Mat image);

  private:
    std::string window_name_;
};

#endif // UNBALL_VISION_GUI_H_
