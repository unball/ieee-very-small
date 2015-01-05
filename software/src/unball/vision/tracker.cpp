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
    {
        cv::namedWindow(window_name_);
        // // The trackbar goes from 0 to 255, wich is the highest number for 8 bit values used by HSV
        // cv::createTrackbar("SMIN", window_name_, &min, 256);
        // cv::createTrackbar("VMIN", window_name_, &max, 256);
    }
}

/**
 * Load field tracking mode, either using the depth or RGB frame.
 */
void Tracker::loadFieldTrackingMode()
{
    std::string field_tracking_mode;
    ros::param::get("/vision/tracker/field_tracking_mode", field_tracking_mode);
    ROS_INFO("Field tracking mode: %s", field_tracking_mode.c_str());

    if (field_tracking_mode != "rgb" and field_tracking_mode != "depth")
    {
        ROS_FATAL("Unknown field tracking mode: %s", field_tracking_mode.c_str());
        exit(BAD_CONFIG);
    }

    tracked_field_.setTrackingMode(field_tracking_mode);
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
 * Finds each robot's center position.
 * @param rgb_segmented_frame OpenCV BGR image after segmentation
 * @param robots_positions The array where each robot's center position will be stored
 */
void Tracker::findRobots(cv::Mat rgb_segmented_frame, cv::Point robots_positions[6])
{
    std::vector< std::vector<cv::Point> > contours;
    cv::Rect tracking_window;
    cv::Point top_left;
    cv::Point bottom_right;
    cv::Point center;

    // Find contours in segmented frame as robots
    cv::findContours(rgb_segmented_frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Calculate each robot's center position
    for (int i = 0; i < contours.size(); ++i)
    {
        // Find bounding rectangle
        tracking_window = cv::boundingRect(contours[i]);

        // Calculate position
        top_left = tracking_window.tl();
        bottom_right = tracking_window.br();
        center.x = (top_left.x + bottom_right.x)/2;
        center.y = (top_left.y + bottom_right.y)/2;
        robots_positions[i] = cv::Point(center);
    }
}

/**
 * Using the original RGB frame and the position of a robot, identifies wich robot
 * the robot_position corresponds to.
 * This function does not work.
 * @param rgb_frame OpenCV BGR image
 * @param robot_position The position of the robot being identified
 * @return The index of the robot, wich acts as its ID.
 */
int Tracker::identifyRobot(cv::Mat rgb_frame, cv::Point robot_position)
{
    // double radius = 10;
    // double angle = 0;
    // double num_points = 20;
    // cv::Point p;
    // cv::Mat hsv, pink, red, tmp, erode_tmp;
    // int red_counter = 0, pink_counter = 0;

    // // Convert to HSV, in order to make color identification easier
    // cv::cvtColor(rgb_frame, hsv, CV_BGR2HSV);
    
    // // inRange for red
    // cv::inRange(hsv, cv::Scalar(170, 170, 170), cv::Scalar(180, 256, 256), tmp);
    // cv::erode(tmp, erode_tmp, cv::Mat::ones(3, 3, CV_8U));
    // cv::dilate(erode_tmp, red, cv::Mat::ones(10, 10, CV_8U));
    // cv::imshow("red", red);

    // // inRange for pink
    // cv::inRange(hsv, cv::Scalar(160, 80, 210), cv::Scalar(256, 140, 256), tmp);
    // cv::erode(tmp, erode_tmp, cv::Mat::ones(3, 3, CV_8U));
    // cv::dilate(erode_tmp, pink, cv::Mat::ones(10, 10, CV_8U));
    // cv::imshow("pink", pink);

    // while (angle < 2*M_PI)
    // {
    //     // Update position being analyzed
    //     p.y = robot_position.y + radius * sin(angle);
    //     p.x = robot_position.x + radius * cos(angle);

    //     // If the robot is the red and yellow one, return 1
    //     if (red.at<uchar>(p.x, p.y) != 0)
    //         ++red_counter;

    //     // If the robot is the pink and yellow one, return 0;
    //     if (pink.at<uchar>(p.x, p.y) != 0)
    //         ++pink_counter;

    //     ROS_ERROR("Red value: %d", red.at<uchar>(p.x, p.y));
    //     ROS_ERROR("Pink value: %d", pink.at<uchar>(p.x, p.y));

    //     cv::circle(rgb_frame, p, 4, cv::Scalar(229, 138, 133));
    //     angle += (2*M_PI)/num_points;
    // }

    // ROS_ERROR("Red counter: %d || Pink counter: %d", red_counter, pink_counter);
    // return 0;
}

/**
 * Track objects in RGB and depth images
 * @param rgb_frame OpenCV BGR image
 * @param depth_frame OpenCV depth image
 * @param rgb_segmented_frame OpenCV BGR image after segmentation
 */
void Tracker::track(cv::Mat rgb_frame, cv::Mat depth_frame, cv::Mat rgb_segmented_frame)
{
    tracked_field_.track(rgb_frame, depth_frame, rgb_segmented_frame);
    cv::Point robots_positions[6];
    findRobots(rgb_segmented_frame, robots_positions);
   
    for (int i = 0; i < 2; ++i)
    {
        // Note: The identification is not yet working.
        int robot_index = identifyRobot(rgb_frame, robots_positions[i]);
        if (robot_index == -1)
        {
            ROS_ERROR("ERROR! Unable to identify robot.");
            break;
        }
        tracked_robot_[robot_index].setPosition(robots_positions[i]);
        tracked_robot_[robot_index].draw(rgb_frame);

        // ROS_ERROR("Robot %d: (%d, %d)", robot_index, robots_positions[i].x, robots_positions[i].y);
    }

    tracked_field_.draw(rgb_frame);
}