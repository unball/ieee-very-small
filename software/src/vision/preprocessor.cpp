/**
 * @file   preprocessor.cpp
 * @author Matheus Vieira Portela
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision preprocessor module
 */

#include <unball/vision/preprocessor.hpp>
#include <opencv2/highgui/highgui.hpp>

Preprocessor::Preprocessor()
{
    window_name_ = "Preprocessor";
    cv::namedWindow(window_name_);
}

Preprocessor::~Preprocessor()
{
    cv::destroyWindow(window_name_);
}

cv::Mat Preprocessor::preprocess(cv::Mat image)
{
    cv::Mat preprocessed;

    // Smoother images are better for segmentation
    cv::medianBlur(image, preprocessed, 5);

    // Show results
    cv::imshow(window_name_, preprocessed);
    cv::waitKey(1);

    return preprocessed;
}