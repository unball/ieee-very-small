/**
 * @file   show_kinect_camera.cpp
 * @author Matheus Vieira Portela
 *         Gabriel Naves da Silva
 * @date   02/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Show normal camera images
 *
 * Subscribes to the /camera/rgb/image_raw topic and shows images from capture
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

static const std::string RGB_WINDOW = "RGB image";


int main(int argc, char **argv)
{
    // Check if video source has been passed as a parameter
    if (argv[1] == NULL) return 1;

    ros::init(argc, argv, "Camera_Publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);

    // Convert the passed as command line parameter index for the video device to an integer
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    
    // Check if it is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1;

    cv::VideoCapture cap(video_source);
    if(!cap.isOpened()) return 1;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    //ros::Rate loop_rate(3);
    while(nh.ok())
    {
        cap >> frame;
        cv::imshow(RGB_WINDOW, frame);
        if(!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
        ros::spinOnce();
        //loop_rate.sleep();
    }
    
    return 0;
}