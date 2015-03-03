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
    cv::Mat rgb_segmented_frame;

    ROS_DEBUG("Run vision");

    /*
     * When the frame does not have proper size, there is no need to execute the vision algorithms, since they will
     * crash. This is not a bug, however, since it can happen when the system is started and no frame has been sent
     * to the vision yet.
     */
    if (isValidSize(rgb_frame_) and isValidSize(depth_frame_))
    {
        if (not homography_.isHomographyDone())
        {
            homography_.calcHomographyMat(gui_.getRGBPoints());
        }
        else
        {
            rgb_frame_ = homography_.transform(rgb_frame_);
            preprocessor_.preprocess(rgb_frame_, depth_frame_);
            rgb_segmented_frame = segmenter_.segment(rgb_frame_);
            tracker_.track(rgb_frame_, depth_frame_, rgb_segmented_frame);
        }
        gui_.setRGBFrame(rgb_frame_);
        gui_.setDepthFrame(depth_frame_);
        gui_.showRGBFrame();
        gui_.showDepthFrame();
    }
}
