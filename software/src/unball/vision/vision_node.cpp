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

void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);
void publishVisionMessage(ros::Publisher &publisher);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(30);
    ros::Publisher publisher = node_handle.advertise<unball::VisionMessage>("vision_topic", 1);

    image_transport::ImageTransport img_transport(node_handle);
    image_transport::Subscriber rgb_sub, depth_sub;

    while (ros::ok())
    {
        Vision::getInstance().run();
        publishVisionMessage(publisher);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * Receives the RGB frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveRGBFrame(const sensor_msgs::ImageConstPtr &msg)
{
    // cv_bridge::CvImagePtr cv_ptr;

    // try
    // {
    //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // }
    // catch (cv_bridge::Exception &e)
    // {
    //     ROS_WARN("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // Vision::getInstance().setRGBFrame(cv_ptr->image);
}

/**
 * Receives the depth frame and passes it to the vision object
 * @param msg a ROS image message pointer.
 */
void receiveDepthFrame(const sensor_msgs::ImageConstPtr &msg)
{
    // cv_bridge::CvImagePtr cv_ptr;

    // try
    // {
    //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    // }
    // catch (cv_bridge::Exception &e)
    // {
    //     ROS_WARN("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // Vision::getInstance().setDepthFrame(cv_ptr->image);
}

/**
 * Publishes the vision message to the vision topic.
 * @param publisher a ROS node publisher.
 */
void publishVisionMessage(ros::Publisher &publisher)
{
    unball::VisionMessage message;
    publisher.publish(message);
}
