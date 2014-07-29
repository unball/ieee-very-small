/**
 * @file   segmenter.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
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
}

Segmenter::~Segmenter()
{
    cv::destroyWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Segmenter::loadConfig()
{
    ROS_INFO("Loading segmenter configurations");
    
    loadShowImage();
    loadHSVMinSConfig();
    loadHSVMinVConfig();
    loadHSVAdjustConfig();
}

/**
 * Load show image configuration.
 */
void Segmenter::loadShowImage()
{
    ros::param::get("/vision/segmenter/hsv_min_s", show_image_);
    ROS_INFO("Saturation show image: %d", hsv_min_s_);

    if (show_image_)
        cv::namedWindow(window_name_);
}

/**
 * Load HSV minimum saturation configuration.
 */
void Segmenter::loadHSVMinSConfig()
{
    ros::param::get("/vision/segmenter/hsv_min_s", hsv_min_s_);
    ROS_INFO("HSV minimum saturation: %d", hsv_min_s_);
}

/**
 * Load HSV minimum value configuration.
 */
void Segmenter::loadHSVMinVConfig()
{
    ros::param::get("/vision/segmenter/hsv_min_v", hsv_min_v_);
    ROS_INFO("HSV minimum value: %d", hsv_min_v_);
}

/**
 * Load HSV adjust configuration. This configuration is used to set the minimum values for saturation and value in HSV
 * segmentation by creating trackbars on the segmenter window.
 */
void Segmenter::loadHSVAdjustConfig()
{
    bool hsv_adjust;
    ros::param::get("/vision/segmenter/hsv_adjust", hsv_adjust);

    ROS_INFO("HSV adjust: %d", hsv_adjust);

    if (hsv_adjust)
    {
        // The trackbar goes from 0 to 255, wich is the highest number for 8 bit values used by HSV
        cv::createTrackbar("SMIN", window_name_, &hsv_min_s_, 256);
        cv::createTrackbar("VMIN", window_name_, &hsv_min_v_, 256);
    }
}

/**
 * Receives an BGR image and apply segmentation to it.
 * First, it converts from the BGR color space to HSV, since it is better for segmentation purposes.
 * Then, it looks for points that lie between (0, s_min_, v_min_) and (180, 256, 256). The values for s_min_ and v_min_
 * are set manually.
 * Finally, it applies a morphological open (erosion followed by dilation) to remove noise.
 *
 * @param image image that will be segmented.
 * @return Black and white segmentation mask.
 */
cv::Mat Segmenter::segment(cv::Mat image)
{
    cv::Mat mask;
    cv::Mat hsv;
    cv::Mat structuring_element;

    // Convert to HSV, which segments faster than HLS and better than BGR for color segmentation
    cv::cvtColor(image, hsv, CV_BGR2HSV);

    /*
     * (180, 256, 256) is the HSV coordinate for cyan. From 0 to 180, we have the following colors:
     * - 0: Red
     * - 30: Orange 
     * - 60: Yellow
     * - 120: Green
     * - 150: Cyan
     * If we need any color that lies out of this scope, we should change the Hue value.
     * This range was chosen due to the colors that are most commonly present in the game field.
     */
    cv::inRange(hsv, cv::Scalar(0, hsv_min_s_, hsv_min_v_), cv::Scalar(180, 256, 256), mask);

    /*
     * Creating a kernel for morphologic transformations. The second parameter is the size of this kernel.
     * Empirically, a kernel of 3x3 generates good results for our application.
     */
    structuring_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    /* The first parameter is the anchor point, which OpenCV defined as (-1, -1) by default. This indicates that the
     * operation will be evaluated with respect to the kernel's center.
     * The second parameter is the number of iterations for this operation. It is faster to apply the morphologic
     * transformation twice than do it one with a larger kernel.
     */
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, structuring_element, cv::Point(-1,-1), 2);

    // TODO(matheus.v.portela@gmail.com): GUI show be the only one to deal with showing images.
    // Show results
    if (show_image_)
    {
        cv::imshow(window_name_, mask);
        cv::waitKey(1);
    }

    return mask;
}
