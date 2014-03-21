/**
 * @file   strategy_node.cpp
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Implements strategy for robots
 * 
 * This node subscribes to the vision topic, applies strategy to decide
 * robots linear and angular velocities, and publishes to the strategy
 * topic
 */

#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

void publish(ros::Publisher &publisher, std::string message);
void receiveVisionMessage(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1000, receiveVisionMessage);
    ros::Publisher publisher = n.advertise<std_msgs::String>("strategy_topic", 1000);
    
    while (ros::ok())
    {
        publish(publisher, "Testing");
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void publish(ros::Publisher &publisher, std::string message)
{
    std_msgs::String msg;
    
    ROS_DEBUG("Publishing: %s", message.c_str());
    
    msg.data = message;
    publisher.publish(msg);
}

void receiveVisionMessage(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG("Receiving: [%s]", msg->data.c_str());
}
