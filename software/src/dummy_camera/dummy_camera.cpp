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

#include <string>

std::string to_string(int num)
{
    std::string result;
    char tmp[100];
    sprintf(tmp, "%d\0", num);
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
    depth_image_file = folder + "depth ";
    int depth_counter = 0;

    // Set the loop rate, defined by the framerate of the video
    ros::Rate loop_rate(rgb_cap.get(CV_CAP_PROP_FPS));
    ROS_DEBUG("Loop rate: %lf", rgb_cap.get(CV_CAP_PROP_FPS));

    // Set rgb and depth frame encoding
    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
//    depth_frame.encoding = sensor_msgs::image_encodings::MONO8;
    
    // Retrieve amount of frames on the video
    num_frames = rgb_cap.get(CV_CAP_PROP_FRAME_COUNT);
    ROS_DEBUG("Frame number: %d", num_frames);

    // Publish the video
    ROS_INFO("Sending video");
    for (frame_counter = 0; ros::ok() && (frame_counter < num_frames); frame_counter++)
    {
        ROS_DEBUG("Frame counter: %d", frame_counter);
        ROS_DEBUG("Publishing the rgb frame");
        rgb_cap >> rgb_frame.image; // Get a new frame from the rgb video capture
        rgb_pub.publish(rgb_frame.toImageMsg());
        
        // TODO: Finish this later (after getting the actual depth images)
        ROS_DEBUG("Publishing the depth frame");
        depth_counter++;
        depth_image_file.erase(depth_image_file.begin()+depth_image_file.size()-1);
        depth_image_file.append(to_string(depth_counter));
        if (depth_counter == 5) depth_counter = 0;
        depth_frame.image = cv::imread(depth_image_file + ".png");
        depth_pub.publish(depth_frame.toImageMsg());
        
        cv::imshow("depth image", depth_frame.image);
        cv::waitKey(3);        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Finished sending video");

    return 0;
}
