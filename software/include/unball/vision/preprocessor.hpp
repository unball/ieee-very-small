/**
 * @file   preprocessor.hpp
 * @author Matheus Vieira Portela
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Definition of the preprocessor class
 */

#ifndef UNBALL_VISION_PREPROCESSOR_H_
#define UNBALL_VISION_PREPROCESSOR_H_

#include <string>
#include <opencv2/opencv.hpp>

class Preprocessor
{
  public:
    Preprocessor();
    ~Preprocessor();
    cv::Mat preprocess(cv::Mat image);

  private:
    std::string window_name_;
};

#endif // UNBALL_VISION_PREPROCESSOR_H_