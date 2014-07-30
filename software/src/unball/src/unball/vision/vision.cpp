/**
 * @file   vision.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Vision class
 *
 * Implements vision, which consists in six parts:
 * - Homography
 * - Pre-processing
 * - Segmentation
 * - Tracking
 * - Communication
 * - GUI
 */

#include <unball/vision/vision.hpp>

/**
 * Vision singleton instance.
 */
Vision* Vision::instance = NULL;

Vision& Vision::getInstance()
{
    if (instance == NULL)
        instance = new Vision();

    return *instance;
}

void Vision::setRGBFrame(cv::Mat rgb_frame)
{
    if (isValidSize(rgb_frame))
        rgb_frame_ = rgb_frame;
    else
        ROS_ERROR("Invalid RGB frame size");
}

void Vision::setDepthFrame(cv::Mat depth_frame)
{
    if (isValidSize(depth_frame))
        depth_frame_ = depth_frame;
    else
        ROS_ERROR("Invalid depth frame size");
}

/**
 * Check whether a frame has a valid size, i.e., neither the width nor the height can be zero.
 * @param frame image that will be checked.
 * @return true if the image has proper size, false otherwise.
 */
bool Vision::isValidSize(cv::Mat frame)
{
    bool result = (frame.rows == 0 or frame.cols == 0) ? false : true;
    return result;
}

/**
 * @param robot_number number of the robot.
 */
std::vector<float> Vision::getRobotPose(int robot_number)
{
    std::vector<float> pose(3);
    
    // Returning dummy values. This should be removed when positions are correctly extracted from the image.
    pose[0] = 0;
    pose[1] = 1;
    pose[2] = 2;

    return pose;
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Vision::loadConfig()
{
    ros::param::get("/vision/using_rgb", using_rgb_);
    ros::param::get("/vision/using_depth", using_depth_);

    preprocessor_.loadConfig();
    segmenter_.loadConfig();
    tracker_.loadConfig();
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    cv::Mat preprocessed;
    cv::Mat segmented;
    
    ROS_DEBUG("Run vision");

    gui_.setRGBFrame(rgb_frame_);
    gui_.setDepthFrame(depth_frame_);

    /*
     * When the frame does not have proper size, there is no need to execute the vision algorithms, since they will
     * crash. This is not a bug, however, since it can happen when the system is started and no frame has been sent
     * to the vision yet.
     */
    if ((using_rgb_) and (isValidSize(rgb_frame_)))
    {
        preprocessed = preprocessor_.preprocessRGB(rgb_frame_);
        segmented = segmenter_.segment(preprocessed);
        tracker_.track(preprocessed, segmented);

        gui_.showRGBFrame();
    }

    if ((using_depth_) and (isValidSize(depth_frame_)))
    {
        preprocessor_.preprocessDepth(depth_frame_);

        gui_.showDepthFrame();
    }

    // Automatically find the soccer field
    if ((using_rgb_) and (isValidSize(rgb_frame_)))
    {
        cv::Mat gray_frame;
        cv::Mat edges_frame;
        cv::Mat contours_frame;
        std::vector< std::vector<cv::Point> > contours;
        std::vector< std::vector<cv::Vec4i> > hierarchy;
        cv::Scalar color;
        cv::RNG rng(12345);

        // Convert to grayscale
        cv::cvtColor(preprocessed, gray_frame, CV_BGR2GRAY);

        // Detect edges
        cv::Canny(gray_frame, edges_frame, 100, 200, 3);

        // Find contours
        cv::findContours(edges_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Draw contours
        contours_frame = cv::Mat::zeros(edges_frame.size(), CV_8UC3);
        for (int i = 0; i < contours.size(); ++i)
        {
            color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            cv::drawContours(contours_frame, contours, i, color);
        }

        gui_.show(contours_frame);
    }
}
