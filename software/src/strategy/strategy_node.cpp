/**
 * @file   strategy_node.cpp
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Run strategy for robots
 * 
 * This node subscribes to the vision topic, applies strategy to decide
 * robots linear and angular velocities, and publishes to the strategy
 * topic
 */

#include <string>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "strategy.hpp"

Strategy strategy;

void publishRobotsVelocities(ros::Publisher &publisher);
void receiveVisionMessage(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1000, receiveVisionMessage);
    ros::Publisher publisher = n.advertise<std_msgs::String>("strategy_topic", 1000);
    
    // TODO(Matheus Vieira Portela): Remove this testing positions to get
    // from vision or simulation. Should be done when vision and simulation
    // nodes are completed.
    float r[6] = {1, 2, 3, 4, 5, 6};
    
    for (int i = 0; i < 6; i++)
        strategy.setRobotLocation(r[i], i);
    
    while (ros::ok())
    {
        strategy.run();
        
        publishRobotsVelocities(publisher);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

/**
 * Publishes the six robots velocities to the strategy topic.
 * Each velocity is separated by a space.
 * Example: 0.2 0.4 0.6 0.8 1.0 1.2
 * 
 * @param publisher a ROS node publisher.
 */
void publishRobotsVelocities(ros::Publisher &publisher)
{
    std_msgs::String message;
    std::ostringstream message_buffer;
    
    for (int i = 0; i < 6; i++)
        message_buffer << strategy.getRobotVelocity(i) << ' ';
    
    message.data = message_buffer.str();
    publisher.publish(message);
    
    ROS_INFO("Publishing: [%s]", message.data.c_str());
}

/**
 * Receives the six robots locations through the vision topic.
 * Each location is separated by a space.
 * Example: 0.2 0.4 0.6 0.8 1.0 1.2
 * 
 * @param msg a ROS string message pointer.
 */
void receiveVisionMessage(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Receiving: [%s]", msg->data.c_str());
    
    // TODO(Matheus Vieira Portela): Remove this testing positions to get
    // from vision or simulation. Should be done when vision and simulation
    // nodes are completed.
    float r[6] = {1, 2, 3, 4, 5, 6};
    
    for (int i = 0; i < 6; i++)
        strategy.setRobotLocation(r[i], i);
}
