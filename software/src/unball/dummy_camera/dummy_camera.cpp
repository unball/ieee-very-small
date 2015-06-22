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
 * Loads the rgb video file and publishes it on the "/camera/rgb/image_raw" topic,
 * and loads the depth images in sequence and publishes them on the
 * "/camera/depth/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <cmath>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher rgb_pub = it.advertise("/camera/rgb/image_raw", 1);
    image_transport::Publisher depth_pub = it.advertise("/camera/depth/image_raw", 1);
    cv_bridge::CvImage rgb_frame, depth_frame;
    int frame_counter; // Used to count the number of frames published on the topic.
    int num_frames;

    // Check if enough arguments where given
    if( argc != 3)
    {
        ROS_ERROR("Not enough arguments. Usage: dummy_camera <rgb video file> <depth image folder>");
        return -1;
    }

    // Load rgb video and check for errors
    cv::VideoCapture rgb_cap(argv[1]);
    if (!rgb_cap.isOpened())
    {
        ROS_ERROR("Could not open of find the rgb video file");
        return -1;
    }

    // Opens depth image and check for errors
    std::string depth_image_file, folder(argv[2]);
    if (depth_image_file[depth_image_file.size()-1] == '/') depth_image_file.erase(depth_image_file.end());
    depth_image_file = folder + "/depth";
    int depth_counter = 0;

    // Set the loop rate, defined by the framerate of the video
    double rgb_video_fps = rgb_cap.get(CV_CAP_PROP_FPS);
    ros::Rate loop_rate(isnan(rgb_video_fps) ? 25 : rgb_video_fps);
    ROS_INFO("Loop rate: %lf", rgb_video_fps);

    // Set rgb and depth frame encoding
    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
    depth_frame.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    // Retrieve amount of frames on the video
    num_frames = rgb_cap.get(CV_CAP_PROP_FRAME_COUNT);
    ROS_INFO("Frame number: %d", num_frames);

    // Publish the video
    ROS_INFO("Sending video");
    for (frame_counter = 0; ros::ok() && (frame_counter < num_frames); frame_counter++)
    {
        ROS_DEBUG("Frame counter: %d", frame_counter);
        // Publish the rgb frame
        ROS_DEBUG("Publishing the rgb frame");
        rgb_cap >> rgb_frame.image; // Get a new frame from the rgb video capture
        rgb_pub.publish(rgb_frame.toImageMsg());

        // Publish the depth frame
        ROS_DEBUG("Publishing the depth frame");
        depth_counter++;
        depth_frame.image = cv::imread(depth_image_file+to_string(depth_counter)+".png", CV_LOAD_IMAGE_ANYDEPTH);
        depth_pub.publish(depth_frame.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Finished sending video");

    return 0;
}
