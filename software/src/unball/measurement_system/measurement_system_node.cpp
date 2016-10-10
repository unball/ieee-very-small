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
    ros::Subscriber sub2 = n.subscribe("simulator_topic", 1, receiveSimulatorMessage);
    publisher = n.advertise<unball::MeasurementSystemMessage>("measurement_system_topic", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void receiveSimulatorMessage(const unball::SimulatorMessage::ConstPtr &msg_s)
{
    std::vector<float> x(6), y(6), th(6);
    std::vector<float> ball_location(2);

    //ROS_INFO("\n\n[MeasurementNode]:ReceiveSimulatorMessage - Receiving simulator message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        //ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, msg_s->x[robot_index], msg_s->y[robot_index],
            //msg_s->th[robot_index]);
        message.x[robot_index] = msg_s->x[robot_index];
        message.y[robot_index] = msg_s->y[robot_index];
        message.th[robot_index] = msg_s->th[robot_index];

    }
    //ROS_INFO("Ball: x: %f, y: %f", ball_location[0], ball_location[1]); message.ball_x = msg_s->ball_x;
    message.ball_y = msg_s->ball_y;

    //ROS_INFO("\n\n[MeasurementNode]:ReceiveSimulatorMessage - Sending measurement system message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        //ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, message.x[robot_index], message.y[robot_index],
            //message.th[robot_index]);
    }
    //ROS_INFO("Ball: x: %f, y: %f", message.ball_x, message.ball_y);

    publisher.publish(message);
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
    convertPixelsToMeters();
    rotateAxis();

    ROS_INFO("Ball: x: %f, y: %f", ball_location[0], ball_location[1]);
    message.ball_x = msg_v->ball_x;
    message.ball_y = msg_v->ball_y;

    ROS_INFO("\n\n[MeasurementNode]:ReceiveVisionMessage - Sending measurement system message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, message.x[robot_index], message.y[robot_index],
            message.th[robot_index]);
    }
    ROS_INFO("Ball: x: %f, y: %f", message.ball_x, message.ball_y);

    publisher.publish(message); 
}

void rotateAxis()
{
    float aux;

    for (int i = 0; i < 6; i++)
    {
        float aux;

        aux = -message.x[i];
        message.x[i] = message.y[i];
        message.y[i] = aux;
        message.th[i] = (message.th[i] > 0 ? message.th[i] : (2*M_PI + message.th[i])) * 360 / (2*M_PI); //convert to degrees
        message.th[i]  -= 90;
        if(message.th[i] < 0)
            message.th[i] += 360;
        if(message.th[i] == 90)
            message.th[i] = M_PI_2;
        else if(message.th[i] == 270)
            message.th[i] = -M_PI_2;
        else
        {
            message.th[i] = tan(message.th[i] * M_PI/180.0);
            message.th[i] = atan2(message.th[i], 1); // convert to radians
        } 
    }
    aux = -message.ball_x;
    message.ball_x = message.ball_y;
    message.ball_y = aux;
}

void convertPixelsToMeters(){
    auto x_conversion = 1.50 / 640;
    auto y_conversion = 1.30 / 480;
    for (int i = 0; i < 6; ++i)
    {
        message.x[i] -= 320;
        message.y[i] -= 240;
        message.x[i] *= x_conversion;
        message.y[i] *= y_conversion;
    }
    message.ball_x -= 320;
    message.ball_y -= 240;
    message.ball_x *= x_conversion;
    message.ball_y *= y_conversion;
}