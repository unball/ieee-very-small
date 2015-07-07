/**
 * @file   tracked_field.cpp
 * @author Matheus Vieira Portela
 * @date   29/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Field class implementation for tracker
 */

#include <unball/vision/tracked_field.hpp>

// Exponential moving average constant
const float TrackedField::AVG_CONSTANT = 0.25;

// Central circle radius in pixels
const int TrackedField::CIRCLE_RADIUS = 60;

// Bounding rectangle color in BGR
const cv::Scalar TrackedField::RECTANGLE_COLOR(0, 255, 0);

// Central circle color in BGR
const cv::Scalar TrackedField::CIRCLE_COLOR(0, 0, 255);

TrackedField::TrackedField()
{
    type_ = FIELD;
    tracking_mode_ = "rgb";
    field_stabilization_counter_ = 0;
    is_field_stable_ = false;
}

TrackedField::~TrackedField()
{
}

void TrackedField::loadConfig()
{
    ros::param::get("/vision/tracker/field_stabilization_frame", field_stabilization_frame_);
    ros::param::get("/vision/tracker/field_center_discrepancy", field_center_discrepancy_);
    loadTrackingMode();
}

void TrackedField::loadTrackingMode()
{
    ros::param::get("/vision/tracker/field_tracking_mode", tracking_mode_);
    ROS_INFO("Field tracking mode: %s", tracking_mode_.c_str());

    if (tracking_mode_ != "rgb" and tracking_mode_ != "depth")
    {
        ROS_FATAL("Unknown field tracking mode: %s", tracking_mode_.c_str());
        exit(BAD_CONFIG);
    }
}

/**
 * Calculates exponential moving average. This filter updates the value by giving more importance to new values than
 * older ones according to a constant value.
 * The average constant indicates the level of confidence in the measured values, from 0 to 1. Higher values indicate
 * that new measurements are relatively precise and should receive more importance than older ones.
 * @param old_value Previous accumulated value
 * @param new_value Measured value
 * @return Averaged value according to the constant
 */
int TrackedField::exponentialMovingAvg(int old_value, int new_value)
{
    return (AVG_CONSTANT*new_value + (1.0-AVG_CONSTANT)*old_value);
}

/**
 * Updates the field position using an exponential moving average.
 * @param position measured position
 */
void TrackedField::updatePosition(cv::Point position)
{
    cv::Point old_position = position_;
    position_.x = exponentialMovingAvg(position_.x, position.x);
    position_.y = exponentialMovingAvg(position_.y, position.y);
    checkFieldStabilization(old_position, position_);
}

/**
 * Updates the field bounding rectangle using an exponential moving average.
 * @param tracking_window new bounding rectangle
 */
void TrackedField::updateTrackingWindow(cv::Rect tracking_window)
{
    tracking_window_.x = exponentialMovingAvg(tracking_window_.x, tracking_window.x);
    tracking_window_.y = exponentialMovingAvg(tracking_window_.y, tracking_window.y);
    tracking_window_.width = exponentialMovingAvg(tracking_window_.width, tracking_window.width);
    tracking_window_.height = exponentialMovingAvg(tracking_window_.height, tracking_window.height);
}

/**
 * Using the previous and the current position of the field center, decides whether the field position is stable or not.
 * @param old_position the old field's position
 * @param new_position the new field's position
 */
void TrackedField::checkFieldStabilization(cv::Point old_position, cv::Point new_position)
{
    if (abs(new_position.x-old_position.x) <= field_center_discrepancy_ and
        abs(new_position.y-old_position.y) <= field_center_discrepancy_)
        ++field_stabilization_counter_;
    else
        field_stabilization_counter_ = 0;

    if (field_stabilization_counter_ >= field_stabilization_frame_)
        is_field_stable_ = true;
}

/**
 * Track soccer field using edge and contour detection.
 * First, it detects the edges using Canny detector. Then it find all contours and calculate the longest one, which
 * should be the soccer field. Finally, it extract the bounding rectangle and update the field center.
 * @param rgb_frame BGR image containing a soccer field
 */
void TrackedField::trackWithRGB(cv::Mat &rgb_frame)
{
    cv::Mat gray_frame;
    cv::Mat edges_frame;
    cv::Mat contours_frame;
    std::vector< std::vector<cv::Point> > contours;
    double contour_length;
    double largest_length = 0;
    int largest_length_index = 0;
    cv::Rect tracking_window;
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
            tracking_window = cv::boundingRect(contours[i]);
        }
    }

    // Calculate field center
    top_left = tracking_window.tl();
    bottom_right = tracking_window.br();
    field_center.x = (top_left.x + bottom_right.x)/2;
    field_center.y = (top_left.y + bottom_right.y)/2;

    // Update
    updatePosition(field_center);
    updateTrackingWindow(tracking_window);
}

/**
 * Track soccer field using edge detection in depth frame. This implementation is really bad and should not be used yet.
 * @param depth_frame OpenCV 16UC1 depth image containing a soccer field
 */
void TrackedField::trackWithDepth(cv::Mat &depth_frame)
{
    cv::Mat binary_frame(depth_frame.rows, depth_frame.cols, CV_8UC1);
    cv::Mat contours_frame;
    std::vector< std::vector<cv::Point> > contours;
    double contour_length;
    double largest_length = 0;
    int largest_length_index = 0;
    cv::Rect tracking_window;
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
            tracking_window = cv::boundingRect(contours[i]);
        }

        cv::drawContours(depth_frame, contours, i, cv::Scalar(0, 0, 255));
    }

    // Calculate field center
    top_left = tracking_window.tl();
    bottom_right = tracking_window.br();
    field_center.x = (top_left.x + bottom_right.x)/2;
    field_center.y = (top_left.y + bottom_right.y)/2;

    // Update
    updatePosition(field_center);
    updateTrackingWindow(tracking_window);
}

void TrackedField::track(cv::Mat &rgb_frame, cv::Mat &depth_frame, cv::Mat &rgb_segmented_frame)
{
    if (not is_field_stable_)
    {
        if (tracking_mode_ == "rgb")
            trackWithRGB(rgb_frame);
        else if (tracking_mode_ == "depth")
            trackWithDepth(depth_frame);
    }
}

/**
 * Draw bounding rectangle and central circle
 * @param frame OpenCV image frame to draw on
 */
void TrackedField::draw(cv::Mat &frame)
{
    cv::rectangle(frame, tracking_window_, RECTANGLE_COLOR);
    cv::circle(frame, position_, CIRCLE_RADIUS, CIRCLE_COLOR); // arbitrary value for circle radius
}

bool TrackedField::isFieldStable()
{
    return is_field_stable_;
}

cv::Point TrackedField::getFieldDimensions()
{
    return cv::Point(tracking_window_.width, tracking_window_.height);
}

