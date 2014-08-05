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
 * Load field tracking mode, either using the depth or RGB frame.
 */
void Tracker::loadFieldTrackingMode()
{
    ros::param::get("/vision/tracker/field_tracking_mode", field_tracking_mode_);
    ROS_INFO("Field tracking mode: %s", field_tracking_mode_.c_str());

    if (field_tracking_mode_ != "rgb" and field_tracking_mode_ != "depth")
    {
        ROS_FATAL("Unknown field tracking mode: %s", field_tracking_mode_.c_str());
        exit(BAD_CONFIG);
    }
}

/**
 * Load configurations.
 * @warning This method must be called after initializing ROS using ros::init in the node main function.
 */
void Tracker::loadConfig()
{
    loadShowImage();
    loadFieldTrackingMode();
}

/**
 * Track soccer field using edge and contour detection.
 * First, it detects the edges using Canny detector. Then it find all contours and calculate the longest one, which
 * should be the soccer field. Finally, it extract the bounding rectangle and update the field center.
 * @param rgb_frame BGR image containing a soccer field
 */
void Tracker::trackFieldWithRGB(cv::Mat rgb_frame)
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
    cv::cvtColor(rgb_frame, gray_frame, CV_BGR2GRAY);
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

    // Update
    tracked_field_.updatePosition(field_center);
    tracked_field_.updateBoundingRect(bounding_rect);

    // Draw results
    tracked_field_.drawMarker(rgb_frame);
}

void Tracker::trackFieldWithDepth(cv::Mat depth_frame)
{
    cv::Mat binary_frame(depth_frame.rows, depth_frame.cols, CV_8UC1);
    cv::Mat contours_frame;
    std::vector< std::vector<cv::Point> > contours;
    double contour_length;
    double largest_length = 0;
    int largest_length_index = 0;
    cv::Rect bounding_rect;
    cv::Point top_left;
    cv::Point bottom_right;
    cv::Point field_center;
    unsigned short int pixel;
    double mean = 0;
    double count = 0;
    int maximum_distance = 2000;
    int threshold = 15;
    cv::Mat structuring_element;

    // Finding average depth value
    for (int i = 0; i < depth_frame.rows; ++i)
    {
        for (int j = 0; j < depth_frame.cols; ++j)
        {
            pixel = depth_frame.at<unsigned short int>(i, j);
            mean += pixel;
        }
    }

    mean /= depth_frame.rows*depth_frame.cols;

    cv::medianBlur(depth_frame, depth_frame, 5);

    // Segment using average
    for (int i = 0; i < depth_frame.rows; ++i)
    {
        for (int j = 0; j < depth_frame.cols; ++j)
        {
            pixel = depth_frame.at<unsigned short int>(i, j);

            if (pixel < (mean + threshold) and pixel > (mean - threshold))
                binary_frame.at<unsigned char>(i, j) = 255;
            else
                binary_frame.at<unsigned char>(i, j) = 0;
        }
    }

    cv::imshow("Binary", binary_frame);
    cv::waitKey(1);

    // Find contours
    cv::findContours(binary_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Find largest contour
    contours_frame = cv::Mat::zeros(binary_frame.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); ++i)
    {
        contour_length = cv::arcLength(contours[i], false);

        if (contour_length > largest_length)
        {
            largest_length = contour_length;
            largest_length_index = i;
            bounding_rect = cv::boundingRect(contours[i]);
        }

        cv::drawContours(depth_frame, contours, i, cv::Scalar(0, 0, 255));
    }

    // Calculate field center
    top_left = bounding_rect.tl();
    bottom_right = bounding_rect.br();
    field_center.x = (top_left.x + bottom_right.x)/2;
    field_center.y = (top_left.y + bottom_right.y)/2;

    // Update
    tracked_field_.updatePosition(field_center);
    tracked_field_.updateBoundingRect(bounding_rect);

    // Draw results
    tracked_field_.drawMarker(depth_frame);
}

/**
 * Track soccer field using either the RGB or depth frames
 * @param rgb_frame OpenCV BGR image containing a soccer field
 * @param depth_frame 16UC1 image from depth sensor
 */
void Tracker::trackField(cv::Mat rgb_frame, cv::Mat depth_frame)
{
    if (field_tracking_mode_ == "rgb")
        trackFieldWithRGB(rgb_frame);
    else if (field_tracking_mode_ == "depth")
        trackFieldWithDepth(depth_frame);
}

/**
 * Track objects in RGB and depth images
 * @param preprocessed OpenCV BGR image
 * @param segmented OpenCV segmented image
 */
void Tracker::track(cv::Mat rgb_frame, cv::Mat depth_frame)
{
    trackField(rgb_frame, depth_frame);
}