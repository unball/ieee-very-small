/**
 * @file   tracker.cpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracker implementation file for vision module
 */

#include <unball/vision/tracker.hpp>

Tracker::Tracker()
{
    window_name_ = "Tracker";
    calculated_measurement_parameters_ = false;
}

Tracker::~Tracker()
{
    if (show_image_)
        cv::destroyWindow(window_name_);
}

/**
 * Load show image. Creates an OpenCV window if show image is set to true.
 */
void Tracker::loadShowImage()
{
    ros::param::get("/vision/tracker/show_image", show_image_);
    ROS_INFO("Tracker show image: %d", show_image_);

    if (show_image_)
        cv::namedWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Tracker::loadConfig()
{
    loadShowImage();
    tracked_field_.loadConfig();
    measurement_conversion_.loadConfig();
    robot_tracker_.loadConfig();
}

/**
 * Track objects in RGB and depth images
 * @param preprocessed OpenCV BGR image
 * @param segmented OpenCV segmented image
 */
void Tracker::track(cv::Mat rgb_frame, cv::Mat depth_frame, cv::Mat rgb_segmented_frame, cv::Mat depth_segmented_frame)
{
    tracked_field_.track(rgb_frame, depth_frame, rgb_segmented_frame);
    //robot_tracker_.track(rgb_frame, depth_frame, depth_segmented_frame);
    ball_tracker_.track(rgb_frame, rgb_segmented_frame);

    if (tracked_field_.isFieldStable() && !calculated_measurement_parameters_)
        calculateMeasurementConversion();

    tracked_field_.draw(rgb_frame);
    robot_tracker_.draw(rgb_frame);
}

/**
 * Sends the field dimensions to the measurement conversion class
 */
void Tracker::calculateMeasurementConversion()
{
    cv::Point field_dimensions = tracked_field_.getFieldDimensions();
    measurement_conversion_.calculateConversion(field_dimensions.x, field_dimensions.y);
    measurement_conversion_.setFieldCenter(tracked_field_.getFieldCenter());
    calculated_measurement_parameters_ = true;
}

std::vector<float> Tracker::getRobotPose(int robot_index)
{
    return robot_tracker_.getRobotPose(robot_index);
}

cv::Point2f Tracker::getBallPose(){
    return ball_tracker_.getBallPose();
}