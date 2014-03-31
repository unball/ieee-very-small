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

#include <vector>

#include <ros/ros.h>

#include "unball/VisionMessage.h"
#include "unball/StrategyMessage.h"
#include "strategy.hpp"

Strategy strategy;

void publishRobotsVelocities(ros::Publisher &publisher);
void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1000, receiveVisionMessage);
    ros::Publisher publisher = n.advertise<unball::StrategyMessage>("strategy_topic", 1000);
    
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
    unball::StrategyMessage msg;
    std::vector<float> velocities;
    
    ROS_DEBUG("Publishing strategy message");
    
    for (int i = 0; i < 6; i++)
    {
        velocities = strategy.getRobotVelocities(i);
        msg.lin_vel[i] = velocities[0];
        msg.rot_vel[i] = velocities[1];
        
        ROS_DEBUG("lin_vel: %f\t rot_vel: %f", msg.lin_vel[i], msg.rot_vel[i]);
    }
    
    publisher.publish(msg);
}

/**
 * Receives the six robots locations through the vision topic.
 * Each location is separated by a space.
 * Example: 0.2 0.4 0.6 0.8 1.0 1.2
 * 
 * @param msg a ROS string message pointer.
 */
void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg)
{
    ROS_DEBUG("Receiving vision message");
    
    for (int i = 0; i < 6; i++)
    {
        ROS_DEBUG("x: %f\t y: %f\t th: %f", msg->x[i], msg->y[i], msg->y[i]);
        strategy.setRobotLocation(msg->x[i], msg->y[i], i);
    }
}
