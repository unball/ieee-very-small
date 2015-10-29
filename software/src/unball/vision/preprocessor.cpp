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
}

Preprocessor::~Preprocessor()
{
    if (show_image_)
        cv::destroyWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Preprocessor::loadConfig()
{
    ros::param::get("/vision/preprocessor/show_image", show_image_);
    ROS_INFO("Preprocessor show image: %d", show_image_);

    if (show_image_)
        cv::namedWindow(window_name_);
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

    // TODO(matheus.v.portela@gmail.com): GUI should be the only one to deal with showing images.
    // Show results
    if (show_image_)
    {
        cv::imshow(window_name_, rgb_frame);
        cv::waitKey(1);
    }
}

/**
 * TODO (gabri.navess@gmail.com): This is a temporary method.
 * It is here so I don't forget how to do this. ;P
 * It will be removed later.
 *
 * @param image the image to be preprocessed
 */
void Preprocessor::printMeanMinMax(const cv::Mat &image)
{
    typedef float pixelType;
    pixelType min = 255, max = 0;
    pixelType mean = 0, counter = 0;
    int npixels = 0;

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            npixels++;
            if (image.at<pixelType>(i, j) != 0)
            {
                counter++;
                mean += image.at<pixelType>(i, j);

                if (image.at<pixelType>(i, j) < min)
                    min = image.at<pixelType>(i, j);

                if (image.at<pixelType>(i, j) > max)
                    max = image.at<pixelType>(i, j);
            }
        }
    }

    if (counter != 0)
    {
        ROS_ERROR("Analyzed %d pixels", (int)counter);
        ROS_ERROR("Mean: %.3f", (float)mean / (float)counter);
    }
    ROS_ERROR("Max: %d", (int)max);
    ROS_ERROR("Min: %d", (int)min);
    ROS_ERROR("Npixels: %d", (int)npixels);
    ROS_ERROR("image dimensions: (%d,%d)", (int)image.cols, (int)image.rows);
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
            if (image.at<uchar>(i, j) == 0)
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
