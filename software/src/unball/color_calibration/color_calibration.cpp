/**
 * @file   color_calibration.cpp
 * @author Gabriel Naves da Silva
 * @date   01/08/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Color calibration class
 *
 * Implements the color calibration system.
 */

#include <unball/color_calibration/color_calibration.hpp>

ColorCalibration* ColorCalibration::instance = NULL;

ColorCalibration& ColorCalibration::getInstance()
{
    if (instance == NULL)
        instance = new ColorCalibration();

    return *instance;
}

ColorCalibration::ColorCalibration()
{
    rgb_window_name_ = "rgb image";
    resulting_window_name_ = "resulting image";
    trackbar_window_name_ = "trackbars";
    hsv_min_h_ = hsv_min_s_ = hsv_min_v_ = 0;
    hsv_max_h_ = hsv_max_s_ = hsv_max_v_ = 255;

    cv::namedWindow(rgb_window_name_);
    cv::namedWindow(resulting_window_name_);
    cv::namedWindow(trackbar_window_name_, CV_WINDOW_NORMAL);

    cv::createTrackbar("HSV minimum hue", trackbar_window_name_, &hsv_min_h_, 255);
    cv::createTrackbar("HSV maximum hue", trackbar_window_name_, &hsv_max_h_, 255);
    cv::createTrackbar("HSV minimum sat", trackbar_window_name_, &hsv_min_s_, 255);
    cv::createTrackbar("HSV maximum sat", trackbar_window_name_, &hsv_max_s_, 255);
    cv::createTrackbar("HSV minimum val", trackbar_window_name_, &hsv_min_v_, 255);
    cv::createTrackbar("HSV maximum val", trackbar_window_name_, &hsv_max_v_, 255);

    has_image_ = false;
}

void ColorCalibration::init(std::string color_name)
{
    color_name_ = color_name;
}

void ColorCalibration::run()
{
    if (has_image_)
    {
        makeResultingImage();
        cv::imshow(rgb_window_name_, rgb_image_);
        cv::imshow(resulting_window_name_, resulting_image_);
        cv::waitKey(1);
    }
}

void ColorCalibration::setImage(cv::Mat rgb_image)
{
    has_image_ = true;
    rgb_image_ = rgb_image;

}

void ColorCalibration::makeResultingImage()
{
    cv::Mat hsv;
    cv::cvtColor(rgb_image_, hsv, CV_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_),
                cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_),
                resulting_image_);
}
