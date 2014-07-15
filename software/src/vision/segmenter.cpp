/**
 * @file   segmenter.cpp
 * @author Matheus Vieira Portela
 * @date   12/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the segmenter class
 */

#include <unball/vision/segmenter.hpp>

Segmenter::Segmenter()
{
    window_name_ = "Segmentation";
    cv::namedWindow(window_name_);

    // Used for training
    //cv::createTrackbar("SMIN", window_name_, &s_min_,256);
    //cv::createTrackbar("VMIN", window_name_, &v_min_,256);

    // Empirical values
    s_min_ = 81;
    v_min_ = 186;
}

Segmenter::~Segmenter()
{
    cv::destroyWindow(window_name_);
}

/**
 * Receives an BGR image and apply segmentation to it.
 * First, it converts from the BGR color space to HSV, since it is better for segmentation purposes.
 * Then, it looks for points that lie between (0, s_min_, v_min_) and (180, 256, 256). The values for s_min_ and v_min_
 * are set manually.
 * Finally, it applies a morphological open (erosion followed by dilation) to remove noise.
 *
 * @param image Image that will be segmented.
 * @return Black and white segmentation mask.
 */
cv::Mat Segmenter::segment(cv::Mat image)
{
    cv::Mat mask;
    cv::Mat hsv;
    cv::Mat structuring_element;

    // Convert to HSV, which segments faster than HLS and better than BGR
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, s_min_, v_min_), cv::Scalar(180, 256, 256), mask);

    // It is faster to apply the morphologic transformation twice than do it one with a
    // larger structuring element
    structuring_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, structuring_element, cv::Point(-1,-1), 2);

    // Show results
    cv::imshow(window_name_, mask);
    cv::waitKey(1);

    return mask;
}