/**
 * @file   show_dummy_camera.cpp
 * @author Gabriel Naves da Silva
 * @author Matheus Vieira Portela
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Show dummy camera images
 *
 * Subscribes to the /camera/image_raw topic and shows each
 * image published by the dummy_camera node.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
int frame_counter = 0;

/**
 * Callback function for the image subscriber.
 * @param msg the message received.
 */
void imageCb(const sensor_msgs::ImageConstPtr &msg)
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
        exit(1);
    }
    
    ROS_DEBUG("Frame counter: %d", frame_counter);
    frame_counter++;
    
    //Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_dummy_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub;
    
    sub = it.subscribe("/camera/image_raw", 1, imageCb); // Subscribes to /camera/image_raw topic
    
    cv::namedWindow(OPENCV_WINDOW);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}

