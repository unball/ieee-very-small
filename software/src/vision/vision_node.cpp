/**
 * @file   vision_node.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Run computer vision
 * 
 * This node subscribes to the camera or dummy_camera topic, applies
 * computer vision algorithms to extract robots positions and publishes
 * to the vision topic
 */

#include <string>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "vision.hpp"

Vision vision;

void publishRobotsLocations(ros::Publisher &publisher);
void receiveCameraFrame(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Rate loop_rate(10);
    
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, receiveCameraFrame);
    ros::Publisher publisher = n.advertise<std_msgs::String>("vision_topic", 1000);
    
    while (ros::ok())
    {
        vision.run();
        
        publishRobotsLocations(publisher);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

/**
 * Publishes the six robots locations to the vision topic.
 * Each location is separated by a space.
 * Example: 0.2 0.4 0.6 0.8 1.0 1.2
 * 
 * @param publisher a ROS node publisher.
 */
void publishRobotsLocations(ros::Publisher &publisher)
{
    std_msgs::String message;
    std::ostringstream message_buffer;
    
    for (int i = 0; i < 6; i++)
        message_buffer << vision.getRobotLocation(i) << ' ';
    
    message.data = message_buffer.str();
    publisher.publish(message);
    
    ROS_DEBUG("Publishing: [%s]", message.data.c_str());
}

/**
 * Receive image frame from camera and give it to
 * the vision object
 * 
 * @param msg a ROS image message pointer.
 */
void receiveCameraFrame(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    vision.setCameraFrame(*cv_ptr);
}
