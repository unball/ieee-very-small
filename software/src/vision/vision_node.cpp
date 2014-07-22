/**
 * @file   vision_node.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Computer vision node
 * 
 * This node subscribes to the camera or dummy_camera topic, applies
 * computer vision algorithms to extract robots positions and publishes
 * to the vision topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <unball/VisionMessage.h>
#include <unball/vision/vision.hpp>

// TODO: Implement this with singleton
Vision vision;

void publishRobotsPoses(ros::Publisher &publisher);
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(10);
    ros::Publisher publisher = node_handle.advertise<unball::VisionMessage>("vision_topic", 1);
    
    image_transport::ImageTransport img_transport(node_handle);
    image_transport::Subscriber rgb_sub, depth_sub;
    
    // TODO: Create a function to do this
    // Load configurations
    bool using_rgb, using_depth;
    
    ros::param::get("/vision/using_rgb", using_rgb);
    if (using_rgb)
        rgb_sub = img_transport.subscribe("camera/rgb/image_raw", 1, receiveRGBFrame);
        
    ros::param::get("/vision/using_depth", using_depth);   
    if (using_depth)
        depth_sub = img_transport.subscribe("camera/depth/image_raw", 1, receiveDepthFrame);
    
    vision.loadConfig();
    
    // Main loop
    while (ros::ok())
    {
        vision.run();
        publishRobotsPoses(publisher);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

/**
 * Publishes the six robots poses (x, y, theta) to the vision topic.
 * @param publisher a ROS node publisher.
 */
void publishRobotsPoses(ros::Publisher &publisher)
{
    unball::VisionMessage message;
    
    ROS_DEBUG("Publishing robots poses");
    
    // TODO: Include y and theta in the loop
    for (unsigned int i = 0; i < message.x.size(); ++i)
        message.x[i] = vision.getRobotPose(i);
    
    publisher.publish(message);
}

/**
 * Receives the RGB frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveRGBFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_WARN("cv_bridge exception: %s", e.what());
        return;
    }
    
    vision.setRGBFrame(cv_ptr->image);
}

/**
 * Receives the depth frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveDepthFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_WARN("cv_bridge exception: %s", e.what());
        return;
    }

    vision.setDepthFrame(cv_ptr->image);
}
