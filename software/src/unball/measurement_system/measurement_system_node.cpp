/**
 * @file   measurement_systen_node.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   23/09/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Rotate robot position datas acording to camera position in the vision and simulator and publishes that.
 *
*/
 
#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <unball/VisionMessage.h>
#include <unball/SimulatorMessage.h>
#include <unball/MeasurementSystemMessage.h>
#include <opencv2/opencv.hpp>

const float field_x_lenght = 1.50;
const float field_y_lenght = 1.30;

const float camera_x_lenght = 640;
const float camera_y_lenght = 480;

void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg_s);
void rotateAxis();
void convertPixelsToMeters();
void receiveSimulatorMessage(const unball::SimulatorMessage::ConstPtr &msg_v);
void publishMeasurementSystemMessage(ros::Publisher &publisher);

unball::MeasurementSystemMessage message;

ros::Publisher publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "measurement_system_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // Hz
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1, receiveVisionMessage);
    publisher = n.advertise<unball::MeasurementSystemMessage>("measurement_system_topic", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg_v)
{
    std::vector<float> x(6), y(6), th(6);
    std::vector<float> ball_location(2);

    ROS_INFO("\n\n[MeasurementNode]:ReceiveVisionMessage - Receiving vision message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, msg_v->x[robot_index], msg_v->y[robot_index],
            msg_v->th[robot_index]);
        message.x[robot_index] = msg_v->x[robot_index];
        message.y[robot_index] = msg_v->y[robot_index];
        message.th[robot_index] = msg_v->th[robot_index];
    }
    message.ball_x = msg_v->ball_x;
    message.ball_y = msg_v->ball_y;
    convertPixelsToMeters();

    ROS_INFO("\n\n[MeasurementNode]:ReceiveVisionMessage - Sending measurement system message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, message.x[robot_index], message.y[robot_index],
            message.th[robot_index]);
    }
    ROS_INFO("Ball: x: %f, y: %f", message.ball_x, message.ball_y);

    publisher.publish(message); 
}

void convertPixelsToMeters(){
    auto x_conversion = field_x_lenght / camera_x_lenght;
    auto y_conversion = field_y_lenght / camera_y_lenght;
    for (int i = 0; i < 6; ++i)
    {
        message.x[i] -= camera_x_lenght / 2;
        message.y[i] -= camera_y_lenght / 2;
        message.x[i] *= x_conversion;
        message.y[i] *= y_conversion;
    }
    message.ball_x -= camera_x_lenght / 2;
    message.ball_y -= camera_y_lenght / 2;
    message.ball_x *= x_conversion;
    message.ball_y *= y_conversion;
}