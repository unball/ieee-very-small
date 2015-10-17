/**
 * @file   save_kinect_video.cpp
 * @author Gabriel Naves da Silva
 * @date   15/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Saves kinect rgb and depth images on two video files.
 *
 * Subscribes to the /camera/rgb/image_raw and /camera/depth/image_raw topics and
 * creates a video file for each of them, using images given by the freenect_launch node.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <string>

cv::VideoWriter rgb_writer;
//cv::VideoWriter depth_writer;
int depth_counter;
std::string depth_file;
bool is_open_rgb, is_open_depth;

/**
 * Converts an integer to a string.
 *
 * @param num the integer to be converted.
 */
std::string to_string(int num)
{
    std::string result;
    char tmp[100];
    sprintf(tmp, "%d", num);
    result = tmp;
    return result;
}

/**
 * Callback function for the rgb image. Saves each image received
 * as a message on the rgb video, using the rgb video writer.
 *
 * @param msg a ROS image message.
 */
void rgbCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    // Copies the image data to cv_ptr and handles exceptions
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check for invalid image
    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(-1);
    }

    // Opens the rgb video writer if it's not opened yet.
    if (!is_open_rgb)
    {
        rgb_writer.open("rgb_video.avi", CV_FOURCC('P','I','M','1'),
                        25, cv::Size(cv_ptr->image.cols, cv_ptr->image.rows),true);
        if (!rgb_writer.isOpened()) ROS_ERROR("Error! Could not open rgb video writer!");
        else is_open_rgb = true;
    }
    rgb_writer << cv_ptr->image; // Saves the rgb image on the rgb video.
}

/**
 * Callback function for the depth image. Saves each image received
 * as a message on the depth video, using the depth video writer.
 *
 * @param msg a ROS image message.
 */
void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    // Copies the image data to cv_ptr and handles exceptions
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check for invalid image
    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(-1);
    }

    /*
    // Opens the depth video writer if it's not opened yet.
    if (!is_open_depth)
    {
        depth_writer.open("depth_video.avi", CV_FOURCC('P','I','M','1'),
                          25, cv::Size(cv_ptr->image.cols, cv_ptr->image.rows),false);
        if (!depth_writer.isOpened()) ROS_ERROR("Error! Could not open depth video writer!");
        else is_open_depth = true;
    }
    */

    /*
    // Normalizes the depth image and converts it from 16-bit to 6-bit.
    cv::Mat normed;
    normalize(cv_ptr->image, normed, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    depth_writer << normed; // Saves the normed image on the depth video.
    */

    depth_counter++;
    cv::imwrite(depth_file+to_string(depth_counter)+".png", cv_ptr->image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_kinect_video");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_rgb, sub_depth;
    is_open_rgb = false;
    is_open_depth = false;
    depth_counter = 0;
    depth_file = "data/depth/depth";

    sub_rgb = it.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
    sub_depth = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

    ROS_INFO("Saving videos");
    ros::spin();

    return 0;
}
