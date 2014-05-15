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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "unball/VisionMessage.h"
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
    
    image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, receiveCameraFrame);
    image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image_raw", 1, receiveCameraFrame);
    ros::Publisher publisher = n.advertise<unball::VisionMessage>("vision_topic", 1000);
    
    while (ros::ok())
    {
        if (!vision.has_field_center_)
        {
            vision.findFieldCenter();
        }
        
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
    unball::VisionMessage message;
 
    ROS_DEBUG("Publishing robots locations");
    for (int i = 0; i < (int)message.x.size(); i++)
    {
        ROS_DEBUG("Robot %d: %f", i, vision.getRobotLocation(i));
        message.x[i] = vision.getRobotLocation(i);
    }
    
    publisher.publish(message);
}

/**
 * Receives both rgb and depth image frame from camera and gives it to
 * the vision object
 * 
 * @param msg a ROS image message pointer.
 */
void receiveCameraFrame(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if (!(cv_ptr->image.rows) || !(cv_ptr->image.cols))
    {
        ROS_ERROR("cv_ptr error: invalid image frame received");
        exit(1);
    }
    
    if (cv_ptr->encoding == sensor_msgs::image_encodings::BGR8)
        vision.setCameraFrame(*cv_ptr, Vision::RGB_IMAGE);
    else if (cv_ptr->encoding == sensor_msgs::image_encodings::MONO8)
        vision.setCameraFrame(*cv_ptr, Vision::DEPTH_IMAGE);
    else 
        ROS_ERROR("Error: invalid image encoding.");
}

