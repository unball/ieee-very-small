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
    window_name_ = "Segmenter";
    depth_window_name_ = "Segmented Depth Frame";
    // depth_threshold_ = 4;
}

Segmenter::~Segmenter()
{
    if (show_image_)
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
    loadDepthSegmentationConfig();
}

/**
 * Load show image configuration.
 */
void Segmenter::loadShowImage()
{
    ros::param::get("/vision/segmenter/show_image", show_image_);
    ROS_INFO("Saturation show image: %d", show_image_);

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
 * Load the depth segmentation configurations. These configurations may alter the behaviour of the depth segmentation
 * algorithm.
 */
void Segmenter::loadDepthSegmentationConfig()
{
    ros::param::get("/vision/segmenter/show_depth_image", show_depth_image_);
    ros::param::get("/vision/segmenter/depth_adjust", depth_adjust_);
    ros::param::get("/vision/segmenter/depth_treshold", depth_threshold_);
    ros::param::get("/vision/segmenter/size_value", size_value_);
    if (depth_adjust_)
    {
        cv::namedWindow(depth_window_name_);
        cv::createTrackbar("Threshold", depth_window_name_, &depth_threshold_, 50);
        cv::createTrackbar("Size", depth_window_name_, &size_value_, 100);
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
cv::Mat Segmenter::segmentRGB(cv::Mat image)
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
     * transformation multiple times than do it one with a larger kernel.
     * Dilate more times than erode to merge the robot's squares as a single blob.
     */
    cv::morphologyEx(mask, mask, cv::MORPH_ERODE, structuring_element, cv::Point(-1,-1), 3);
    cv::morphologyEx(mask, mask, cv::MORPH_DILATE, structuring_element, cv::Point(-1,-1), 5);

    // TODO(matheus.v.portela@gmail.com): GUI show be the only one to deal with showing images.
    // Show results
    if (show_image_)
    {
        cv::imshow(window_name_, mask);
        cv::waitKey(1);
    }

    return mask;
}

/**
 * Receives a depth image and apply segmentation to it.
 *
 * @param image depth image that will be segmented.
 * @return Black and white segmentation mask.
 */
cv::Mat Segmenter::segmentDepth(cv::Mat image)
{
    cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::Mat image_8_bit;

    /*
     * Adaptive Threshold works only with 8-bit single channel images, so the original depth image needs to be
     * converted to this format. Since it has a smaller range of possible values, it needs to be normalized too.
     */
    cv::normalize(image, image_8_bit, 0, 256, cv::NORM_MINMAX, CV_8UC1);

    /*
     * Applies adaptive threshold to the image. The difference from normal thresholding is that the threshold value
     * is calculated for each pixel.
     * The last two parameters are "blockSize" and "C", respectively. Block size takes odd values, starting from 3.
     * C is a constant subtracted from the weighted mean. Both values are managed on trackbars.
     * More info on < http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#adaptivethreshold >
     */
    cv::adaptiveThreshold(image_8_bit, mask, 256, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                          3+(size_value_*2), depth_threshold_-25);

    if (show_depth_image_)
    {
        cv::imshow(depth_window_name_, mask);
        cv::waitKey(1);
    }

    return mask;
}
