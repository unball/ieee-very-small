/**
 * @file   vision.hpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
 *
 * Defines computer vision class
 */

#ifndef UNBALL_VISION_H_
#define UNBALL_VISION_H_

#include <opencv2/highgui/highgui.hpp>
#include <unball/vision/preprocessor.hpp>
#include <unball/vision/segmenter.hpp>
#include <unball/vision/gui.hpp>

class Vision
{
  public:
    void setRGBFrame(cv::Mat rgb_frame);
    void setDepthFrame(cv::Mat depth_frame);
    bool isValidSize(cv::Mat frame);
    float getRobotPose(int robot_index);
    void loadConfig();
    void run();
    
  private:
    bool using_rgb_;
    bool using_depth_;

    GUI gui_;
    Preprocessor preprocessor_;
    Segmenter segmenter_;

    cv::Mat rgb_frame_;
    cv::Mat depth_frame_;
};

#endif // UNBALL_VISION_H_
