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
}

Tracker::~Tracker()
{
    if (show_image_)
        cv::destroyWindow(window_name_);
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Tracker::loadConfig()
{
    ros::param::get("/vision/tracker/show_image", show_image_);
    ROS_INFO("Tracker show image: %d", show_image_);

    if (show_image_)
        cv::namedWindow(window_name_);
}

/**
 * Track soccer field using edge and contour detection.
 * First, it detects the edges using Canny detector. Then it find all contours and calculate the longest one, which
 * should be the soccer field. Finally, it extract the bounding rectangle and update the field center.
 * @param tracking_frame BGR image containing a soccer field
 */
void Tracker::trackField(cv::Mat tracking_frame)
{
        cv::Mat gray_frame;
        cv::Mat edges_frame;
        cv::Mat contours_frame;
        std::vector< std::vector<cv::Point> > contours;
        double contour_length;
        double largest_length = 0;
        int largest_length_index = 0;
        cv::Rect bounding_rect;
        cv::Point top_left;
        cv::Point bottom_right;
        cv::Point field_center;

        // Detect edges with arbitrary threshold values. Must convert to grayscale first.
        // TODO(matheus.v.portela@gmail.com): What about using the depth frame instead of RGB in grayscale?
        cv::cvtColor(tracking_frame, gray_frame, CV_BGR2GRAY);
        cv::Canny(gray_frame, edges_frame, 150, 200, 3);

        // Find contours
        cv::findContours(edges_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Find largest contour
        contours_frame = cv::Mat::zeros(edges_frame.size(), CV_8UC3);
        for (int i = 0; i < contours.size(); ++i)
        {
            contour_length = cv::arcLength(contours[i], false);

            if (contour_length > largest_length)
            {
                largest_length = contour_length;
                largest_length_index = i;
                bounding_rect = cv::boundingRect(contours[i]);
            }
        }

        // Calculate field center
        top_left = bounding_rect.tl();
        bottom_right = bounding_rect.br();
        field_center.x = (top_left.x + bottom_right.x)/2;
        field_center.y = (top_left.y + bottom_right.y)/2;

        // Update field center with exponential average
        updateFieldCenter(field_center);

        // Draw results on the image
        cv::rectangle(tracking_frame, bounding_rect, cv::Scalar(0, 255, 0));
        cv::circle(tracking_frame, field_center_, 60, cv::Scalar(0, 0, 255)); // arbitrary value for circle radius
}

/**
 * Applys exponential moving average to update field center.
 * @param field_center new value for field center
 */
void Tracker::updateFieldCenter(cv::Point field_center)
{
    const float avg_constant = 0.25;

    field_center_.x = (int)(avg_constant*field_center.x + (1.0 - avg_constant)*field_center_.x);
    field_center_.y = (int)(avg_constant*field_center.y + (1.0 - avg_constant)*field_center_.y);
}

/**
 * @param preprocessed OpenCV BGR image
 * @param segmented OpenCV segmented image
 */
void Tracker::track(cv::Mat preprocessed, cv::Mat segmented)
{
    cv::Mat tracking_frame(preprocessed);

    trackField(tracking_frame);

    if (show_image_)
    {
        cv::imshow(window_name_, tracking_frame);
        cv::waitKey(1);
    }
}