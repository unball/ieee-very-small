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
float const LIN_VEL_CONST = 0.8;
float const ANG_VEL_CONST = 500;

int main(int argc, char **argv)
{
	boost::asio::io_service ios;
	boost::asio::serial_port sp(ios, "/dev/ttyUSB0");
	sp.set_option(boost::asio::serial_port::baud_rate(9600));
	
	ros::init(argc, argv, "communication_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(4); // Hz
    
    ros::Subscriber sub = n.subscribe("strategy_topic", 1, receiveStrategyMessage);

    while (ros::ok())
    {
		int robot_number, right_wheel, left_wheel;
		unsigned char result;

		std::string message;

		//for (int i=0; i<3; i++)
		//{
			int i = 0;
			robot_number = i;
			right_wheel = convert(calculateRightSpeed(i));
			left_wheel = convert(calculateLeftSpeed(i));
			result = ((robot_number << 6) & 0b11000000) | ((left_wheel << 3) & 0b00111000) | (right_wheel & 0b00000111);

			message = boost::lexical_cast<std::string>(result);

			// ROS_ERROR("Robot: %d\tLin: %.3f\tAng: %.3f", robot_number, lin_vel[robot_number], ang_vel[robot_number]);
			// ROS_ERROR("Robot: %d\tLeft: %.3f\tRight: %.3f", robot_number, calculateLeftSpeed(i), calculateRightSpeed(i));
			// ROS_ERROR("Robot: %d\tLeft: %d\tRight: %d", robot_number, left_wheel, right_wheel);
			//ROS_ERROR("Message: %d", result);

			sp.write_some(boost::asio::buffer(message, message.size()));
		//}	
        
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
	return (lin_vel[i]*LIN_VEL_CONST - (WHEELS_DISTANCE/2)*ang_vel[i]*ANG_VEL_CONST)/R;
}

float calculateRightSpeed(int i)
{
	return (lin_vel[i]*LIN_VEL_CONST + (WHEELS_DISTANCE/2)*ang_vel[i]*ANG_VEL_CONST)/R;
}

/**
 * Converts the expected speed on the wheels to a number from 0 - 7
 */
int convert(float speed)
{
	//We are assuming the maximum speed is somewhere close to 198. Ranges from 150 and above are on the maximum speed.
	if (speed >= 150)
		return 7;
	else if ((100 <= speed) and (speed < 150))
		return 6;
	else if ((60 <= speed) and (speed < 100))
		return 5;
	else if ((30 <= speed) and (speed < 60))
		return 4;
	else if ((-30 <= speed) and (speed < 30))
		return 3;
	else if ((-80 <= speed) and (speed < -30))
		return 2;
	else if ((-150 <= speed) and (speed < -80))
		return 1;
	else
		return 0;
}