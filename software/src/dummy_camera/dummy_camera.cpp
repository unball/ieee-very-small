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
 * Loads a video file and publishes it on the "camera/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_camera");
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

    // Retrieve amount of frames on the video
    num_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    ROS_DEBUG("Frame number: %d", num_frames);

    // Publish the video
    ROS_INFO("Sending video");
    for (frame_counter = 0; ros::ok() && (frame_counter < num_frames); frame_counter++)
    {
        ROS_DEBUG("Frame counter: %d", frame_counter);
        cap >> frame.image; // Get a new frame from the video capture
        pub.publish(frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Finished sending video");

    return 0;
}

