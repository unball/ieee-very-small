/**
 * @file   communication_node.cpp
 * @author Icaro da Costa Mota
 * @date   25/10/2015
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief
 */

#include <vector>

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <cmath>

#include <unball/StrategyMessage.h>

void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg);
float calculateLeftSpeed(int i);
float calculateRightSpeed(int i);

float lin_vel[3];
float ang_vel[3];

float const R = 0.03;
float const WHEELS_DISTANCE = 0.075;

int main(int argc, char **argv)
{
	boost::asio::io_service ios;
	boost::asio::serial_port sp(ios, "/dev/ttyUSB0");
	sp.set_option(boost::asio::serial_port::baud_rate(250000));
	
	ros::init(argc, argv, "communication_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(30); // Hz
    
    ros::Subscriber sub = n.subscribe("strategy_topic", 1, receiveStrategyMessage);

    while (ros::ok())
    {
		char robot_number, right_wheel, left_wheel;
		char robot_number_char, right_wheel_char, left_wheel_char;
		std::string message;

		for (int i = 0; i < 3; i++)
		{
			robot_number = i + 48;
			right_wheel = (int)calculateRightSpeed(i) + 48;
			left_wheel = (int)calculateLeftSpeed(i) + 48;

			message.push_back(robot_number); message.push_back(right_wheel); message.push_back(left_wheel);

			ROS_ERROR("Robot: %c\tLeft: %c\tRight: %c", i, left_wheel, right_wheel);

			sp.write_some(boost::asio::buffer(message, message.size()));
		}	
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;    
}

void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg)
{
    ROS_DEBUG("Receiving Strategy message");
    
    for (int i = 0; i < 3; i++)
    {
        ROS_DEBUG("%d lin_vel: %f\t ang_vel: %f\t", i, msg->lin_vel[i], msg->ang_vel[i]);
        lin_vel[i] = msg->lin_vel[i];
        ang_vel[i] = msg->ang_vel[i];
    }
}

float calculateLeftSpeed(int i)
{
	return 100;
	float linear_speed_rpm = convertSpeedToRpm(lin_vel[i]);
	float tangential_speed = ang_vel[i]*(WHEELS_DISTANCE/2);
	float tangential_speed_rpm = convertSpeedToRpm(tangential_speed);
	return linear_speed_rpm - tangential_speed_rpm;
}

float calculateRightSpeed(int i)
{
	return 70;
	float linear_speed_rpm = convertSpeedToRpm(lin_vel[i]);
	float tangential_speed = ang_vel[i]*(WHEELS_DISTANCE/2);
	float tangential_speed_rpm = convertSpeedToRpm(tangential_speed);
	return linear_speed_rpm + tangential_speed_rpm;
}

float convertSpeedToRpm(float speed) {
	float wheel_lenght = 2*M_PI*R;
	float rotations_per_second = speed/wheel_lenght;
	float rpm = rotations_per_second * 60;
	return rpm;
}