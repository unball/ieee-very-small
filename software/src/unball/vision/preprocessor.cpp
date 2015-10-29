/**
 * @file   preprocessor.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   15/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Implementation of the computer vision preprocessor module
 */

#include <unball/vision/preprocessor.hpp>

Preprocessor::Preprocessor()
{
    window_name_ = "Preprocessor";
    is_field_calibration_done_ = false;
    has_main_field_ = false;
    has_goal_ = false;
}

Preprocessor::~Preprocessor()
{
    if (show_depth_image_)
        cv::destroyWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Preprocessor::loadConfig()
{
    ros::param::get("/vision/preprocessor/show_depth_image", show_depth_image_);
    ros::param::get("/vision/preprocessor/adjust_noise_reduction", adjust_noise_reduction_);
    ros::param::get("/vision/preprocessor/noise_thresh", noise_thresh_);

    if (show_depth_image_)
    {
        cv::namedWindow(window_name_);
        if (adjust_noise_reduction_)
            cv::createTrackbar("Noise thresh", window_name_, &noise_thresh_, 100);
    }
}

/**
 * Preprocessing is simply applying a median blur to smooth out the image and, afterwards, get better segmentation
 * results.
 */
void Preprocessor::preprocessRGB(cv::Mat &rgb_frame)
{
    /*
     * Smoother images are better for segmentation
     * The last parameter is the aperture linear size K, which must be an odd number. A kernel of size K x K will be
     * applied to the image.
     */
    cv::medianBlur(rgb_frame, rgb_frame, 5);
}

/**
 * Applies a median blur to depth frame, and normalizes it to 8 bits. Also, gets rid of the kinect noise.
 *
 * @param image the depth image to be preprocessed, it has to be a 16-bit unsigned image with one dimension (16UC1)
 */
void Preprocessor::preprocessDepth(cv::Mat &depth_frame)
{
    cv::medianBlur(depth_frame, depth_frame, 5);
    cv::normalize(depth_frame, depth_frame, 0, 256, cv::NORM_MINMAX, CV_8UC1);
    fixDepthImageNoise(depth_frame);

    if (show_depth_image_)
        cv::imshow(window_name_, depth_frame);
}

/**
 * Attempts to remove kinect depth image noise by making the affected pixels be as far as possible instead of
 * near.
 *
 * @param image the depth image to have its noise fixed.
 */
void Preprocessor::fixDepthImageNoise(cv::Mat &image)
{
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
            if (image.at<uchar>(i, j) <= noise_thresh_)
                image.at<uchar>(i, j) = 255;
}

/**
 * Executing preprocessing
 * @param rgb_frame OpenCV BGR frame
 * @param depth_frame OpenCV depth frame, in CV_16UC1 with pixels representing distances in milimeters
 */
void Preprocessor::preprocess(cv::Mat &rgb_frame, cv::Mat &depth_frame)
{
    preprocessRGB(rgb_frame);
    preprocessDepth(depth_frame);
}

void Preprocessor::runFieldCalibration(std::vector<cv::Point2f> rgb_points)
{
    if (not has_main_field_)
        getMainPolygon(rgb_points);
    else if (not has_goal_)
        getGoalPolygon(rgb_points);
    else
        calculateMask();
}

void Preprocessor::getMainPolygon(std::vector<cv::Point2f> rgb_points)
{
    if (rgb_points.size() != 8)
        return;
    main_polygon_.setPoints(rgb_points);
    has_main_field_ = true;
    GUI::clearRGBPoints();
}

void Preprocessor::getGoalPolygon(std::vector<cv::Point2f> rgb_points)
{
    if (rgb_points.size() != 4)
        return;
    goal_polygon_.setPoints(rgb_points);
    has_goal_ = true;
    GUI::clearRGBPoints();
}

void Preprocessor::calculateMask()
{
    field_mask_ = cv::Mat::zeros(480, 640, CV_8UC1);
    for (int i = 0; i < field_mask_.rows; ++i)
    {
        for (int j = 0; j < field_mask_.cols; ++j)
        {
            cv::Point2f point(i, j);
            // if (not( main_polygon_.isPointInside(point) or goal_polygon_.isPointInside(point) ))
            if (main_polygon_.isPointInside(point) or goal_polygon_.isPointInside(point))
                field_mask_.at<uchar>(i, j) = 255;
        }
    }
    is_field_calibration_done_ = true;
}

bool Preprocessor::isFieldCalibrationDone()
{
    return is_field_calibration_done_;
}

cv::Mat Preprocessor::getFieldMask()
{
    return field_mask_;
}
