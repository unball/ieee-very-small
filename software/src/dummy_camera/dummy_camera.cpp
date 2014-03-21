/**
 * @file   dummy_camera.cpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Dummy camera node
 * Loads an image file and publishes it on the "camera/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
    cv_bridge::CvImage frame;

    frame.encoding = sensor_msgs::image_encodings::BGR8; // Sets the encoding of the frame.
    frame.image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR); // Loads the image given as an argument.

    // Publishes the image at a rate of 1 message per second.
    ros::Rate loop_rate(1);
    while (nh.ok()) {
        pub.publish(frame.toImageMsg());
        loop_rate.sleep();
    }

    return 0;
}

