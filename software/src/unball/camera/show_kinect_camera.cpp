/**
 * @file   show_kinect_camera.cpp
 * @author Matheus Vieira Portela
 *         Gabriel Naves da Silva
 * @date   02/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Show Kinect RGB and depth images
 *
 * Subscribes to the /camera/rgb/image_raw and /camera/depth/image_raw topics and shows each
 * image published by the freenect_launch node.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

static const std::string RGB_WINDOW = "RGB image";
static const std::string DEPTH_WINDOW = "Depth image";

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

    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(-1);
    }

    //Update GUI Window
    cv::imshow(RGB_WINDOW, cv_ptr->image);
    cv::waitKey(1);
}

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

    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(-1);
    }

    // Normalizes the depth image
//    cv::Mat normed;
//    normalize(cv_ptr->image, normed, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    //Update GUI Window
    cv::Mat normalized_depth_frame;
    cv::normalize(cv_ptr->image, normalized_depth_frame, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
    cv::imshow(DEPTH_WINDOW, cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_rgb, sub_depth;

    sub_rgb = it.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
    sub_depth = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

    cv::namedWindow(RGB_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);
    ros::spin();
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(DEPTH_WINDOW);

    return 0;
}
