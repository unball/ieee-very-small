/**
 * @file   vision_node.cpp
 * @author Matheus Vieira Portela
 * @date   21/03/2014
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

#include "vision.hpp"

Vision vision;

void publishRobotsLocations(ros::Publisher &publisher);
void receiveVisionMessage(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("camera/image_raw", 1000, receiveVisionMessage);
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
 * Receives the six robots locations through the vision topic.
 * Each location is separated by a space.
 * Example: 0.2 0.4 0.6 0.8 1.0 1.2
 * 
 * @param msg a ROS string message pointer.
 */
void receiveVisionMessage(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG("Receiving: [%s]", msg->data.c_str());
}
