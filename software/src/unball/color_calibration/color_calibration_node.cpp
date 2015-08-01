/**
 * @file   color_calibration_node.cpp
 * @author Gabriel Naves da Silva
 * @date   01/08/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Computer vision color calibration node.
 *
 * This node is responsible for calibrating hsv ranges for desired colors.
 * The hsv ranges are placed in a file, located in data folder.
 * TODO(gabri.navess@gmail.com): The pattern for the file should be documented.
 */

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <unball/color_calibration/color_calibration.hpp>

ColorCalibration color_calibration_;

void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_calibration_node");

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(10);

    image_transport::ImageTransport img_transport(node_handle);
    image_transport::Subscriber rgb_sub;

    rgb_sub = img_transport.subscribe("camera/rgb/image_raw", 1, receiveRGBFrame);

    ColorCalibration::getInstance().init("pink");

    // Main loop
    while (ros::ok())
    {
        ColorCalibration::getInstance().run();
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

    ColorCalibration::getInstance().setImage(cv_ptr->image);
}
