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
 * TODO (matheus.v.portela@gmail.com): Implement this method to return the robot pose (x, y, theta)
 * @param robot_number number of the robot.
 */
float Vision::getRobotPose(int robot_number)
{
    return 0;
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Vision::loadConfig()
{
    ros::param::get("/vision/using_rgb", using_rgb_);
    ros::param::get("/vision/using_depth", using_depth_);

    segmenter_.loadConfig();
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    cv::Mat preprocessed;
    cv::Mat mask;
    
    ROS_DEBUG("Run vision");

    /*
     * When the frame does not have proper size, there is no need to execute the vision algorithms, since they will
     * crash. This is not a bug, however, since it can happen when the system is started and no frame has been sent
     * to the vision yet.
     */
    if ((using_rgb_) and (isValidSize(rgb_frame_)))
    {
        gui_.show(rgb_frame_);
        preprocessed = preprocessor_.preprocess(rgb_frame_);
        mask = segmenter_.segment(preprocessed);
    }

    if ((using_depth_) and (isValidSize(depth_frame_)))
        preprocessor_.preprocessDepth(depth_frame_);
}
