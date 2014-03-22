/**
 * @file   dummy_camera.cpp
 * @author Gabriel Naves da Silva
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Dummy camera node
 * 
 * Loads an image file and publishes it on the "camera/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_camera");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1); // Publishes the image at a rate of 1 message per second.
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
    cv::Mat image;
    cv_bridge::CvImage frame;

    // Check if enough arguments where given
    if( argc != 2)
    {
        ROS_ERROR("Not enough arguments. Usage: dummy_camera <image file>");
        return -1;
    }

    // Load image and check for errors
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    
    if (!image.data)
    {
        ROS_ERROR("Could not open or find the image");
        return -1;
    }
    
    // Set frame encoding and data
    frame.encoding = sensor_msgs::image_encodings::BGR8;
    frame.image = image;

    // Publish image
    while (ros::ok())
    {
        ROS_INFO("Sending image");
        pub.publish(frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

