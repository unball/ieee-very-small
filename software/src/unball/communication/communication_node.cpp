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

int convert(float speed);

float lin_vel[3];
float ang_vel[3];

float const R = 0.03;
float const WHEELS_DISTANCE = 0.075;

int main(int argc, char **argv)
{
	boost::asio::io_service ios;
	boost::asio::serial_port sp(ios, "/dev/ttyUSB0");
	sp.set_option(boost::asio::serial_port::baud_rate(9600));
	
	ros::init(argc, argv, "communication_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(5); // Hz
    
    ros::Subscriber sub = n.subscribe("strategy_topic", 1, receiveStrategyMessage);

    while (ros::ok())
    {
		int robot_number;
		float right_wheel, left_wheel;
		int right_result, left_result;
		char result;

		std::string message;

		for (int i=0; i<3; i++)
		{
			robot_number = i;

			right_wheel = calculateRightSpeed(i);
			left_wheel = calculateLeftSpeed(i);

			robot_number = robot_number << 6;
			left_result = convert(left_wheel) << 3;
			right_result = convert(right_wheel);

			result = robot_number | left_result | right_result;
			
			message = boost::lexical_cast<std::string>(result);
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
	return (lin_vel[i] - (WHEELS_DISTANCE/2)*ang_vel[i])/R;
}

float calculateRightSpeed(int i)
{
	return (lin_vel[i] + (WHEELS_DISTANCE/2)*ang_vel[i])/R;
}

/**
 * Converts the expected speed on the wheels to a number from 0 - 7
 */
int convert(float speed)
{
	//We are assuming the maximum speed is somewhere close to 166. Ranges from 144 and above are on the maximum speed
	if (speed >=0)
		return (speed/36);

	return (speed/48);
}