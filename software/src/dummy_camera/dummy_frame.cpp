/**
 * @file   dummy_frame.cpp
 * @author Matheus Vieira Portela
 * @date   31/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Dummy frame node
 * 
 * Loads a video file and publishes only one frame on the "camera/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_frame");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
    cv_bridge::CvImage frame;
    int frame_counter; // Used to count the number of frames published on the topic.
    int num_frames;

    // Check if enough arguments where given
    if( argc != 2)
    {
        ROS_ERROR("Not enough arguments. Usage: dummy_camera <video file>");
        return -1;
    }

    // Load video and check for errors
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        ROS_ERROR("Could not open of find the video file");
        return -1;
    }

    // Set the loop rate, defined by the framerate of the video
    ros::Rate loop_rate(cap.get(CV_CAP_PROP_FPS));
    ROS_DEBUG("Loop rate: %lf", cap.get(CV_CAP_PROP_FPS));

    // Set frame encoding
    frame.encoding = sensor_msgs::image_encodings::BGR8;

    // Get second frame for publishing. Ignore first frame because it is weird.
    // TODO: Update whether the first frame is weird after creating the Kinect node.
    cap.grab();
    cap >> frame.image;

    // Publish the video
    ROS_INFO("Publishing frame");
    while (ros::ok())
    {
        pub.publish(frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

