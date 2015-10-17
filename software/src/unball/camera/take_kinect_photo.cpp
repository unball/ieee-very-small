/**
 * @file   take_kinect_photo.cpp
 * @author Gabriel Naves da Silva
 * @date   17/10/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Takes a photo using the kinect rgb camera.
 *
 * Subscribes to the /camera/rgb/image_raw topic and shows the images. Clicking the screen takes a photo.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <string>

cv::Mat rgb_frame;
std::string rgb_image_name, window_name;
int photo_amount = 0;

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

    // Check for invalid imagename
    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(-1);
    }

    rgb_frame = cv_ptr->image;
    cv::imshow(window_name, rgb_frame);
    cv::waitKey(1);
}

static void onMouse( int event, int x, int y, int, void* )
{
    if (event != cv::EVENT_LBUTTONDOWN)
        return;
    ROS_INFO("taking pictures of you");
    std::string name = rgb_image_name;
    name += to_string(photo_amount++);
    name += ".png";
    cv::imwrite(name, rgb_frame);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_kinect_video");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_rgb;

    sub_rgb = it.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
    rgb_image_name = ros::package::getPath("unball");
    rgb_image_name += "/data/RBG_photos/image";

    window_name = "RGB image";
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, onMouse);

    ROS_INFO("Running");
    ros::spin();

    return 0;
}
