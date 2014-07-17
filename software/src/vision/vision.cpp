/**
 * @file   vision.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
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
 * Set the vision node handle pointer, which can be used to any ROS feature that requires ros::init, such as parameters
 * parsing.
 * Also, set the node handle to all objects that requires it.
 * @param n Initialized node handle
 */
void Vision::setNodeHandle(ros::NodeHandle *n)
{
    n_ = n;
    segmenter_.setNodeHandle(n);
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
 * @param frame Image that will be checked.
 * @return true is the image has proper size, false otherwise.
 */
bool Vision::isValidSize(cv::Mat frame)
{
    if (frame.rows == 0 || frame.cols == 0)
        return false;
    else
        return true;
}

/**
 * TODO (matheus.v.portela@gmail.com): Implement this method to return the robot pose (x, y, theta)
 */
float Vision::getRobotPose(int robot_number)
{
    return 0;
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    cv::Mat preprocessed;
    cv::Mat mask;

    ROS_DEBUG("Run vision");

    // When the frame does not have proper size, there is no need to execute the vision algorithms, since they will
    // crash. This is not a bug, however, since it can happen when the system is started and no frame has been sent
    // to the vision yet.
    if (not isValidSize(rgb_frame_) || not isValidSize(depth_frame_))
        return;

    gui_.show(rgb_frame_);

    preprocessor_.preprocessDepth(depth_frame_); // Preprocesses the depth image
    preprocessed = preprocessor_.preprocess(rgb_frame_);
    mask = segmenter_.segment(preprocessed);
}
