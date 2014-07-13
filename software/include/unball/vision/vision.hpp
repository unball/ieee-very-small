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
#include <unball/vision/gui.hpp>

class Vision
{
  public:
    void run();
    float getRobotPose(int robot_index);
    void setRGBFrame(cv::Mat rgb_frame);
    void setDepthFrame(cv::Mat depth_frame);
    void checkFrameSize(cv::Mat frame);
    
  private:
    GUI gui_;

    cv::Mat rgb_frame_;
    cv::Mat depth_frame_;
};

#endif // UNBALL_VISION_H_
