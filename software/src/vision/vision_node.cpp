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
#include <cv_bridge/cv_bridge.h>

#include <unball/VisionMessage.h>
#include <unball/vision/vision.hpp>

Vision vision;

void publishRobotsPoses(ros::Publisher &publisher);
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Rate loop_rate(10);
    
    image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, receiveRGBFrame);
    //image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image_raw", 1, receiveDepthFrame);
    ros::Publisher publisher = n.advertise<unball::VisionMessage>("vision_topic", 1);

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

    for (int i = 0; i < (int)message.x.size(); i++)
        message.x[i] = vision.getRobotPose(i);
    
    publisher.publish(message);
}

/**
 * Receives the RGB frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg)
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

    vision.setRGBFrame(cv_ptr->image);
}

/**
 * Receives the depth frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    vision.setDepthFrame(cv_ptr->image);
}
