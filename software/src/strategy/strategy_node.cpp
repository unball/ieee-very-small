/**
 * @file   strategy_node.cpp
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Run strategy for robots
 * 
 * This node subscribes to the vision topic, applies strategy to decide robots linear and angular velocities, and
 * publishes to the strategy topic
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
    ros::Rate loop_rate(10); // 10 Hz
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1, receiveVisionMessage);
    ros::Publisher publisher = n.advertise<unball::StrategyMessage>("strategy_topic", 1);
    
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
 * Publishes the robots velocities to the strategy topic.
 * @param publisher a ROS node publisher.
 */
void publishRobotsVelocities(ros::Publisher &publisher)
{
    unball::StrategyMessage msg;
    std::vector<float> velocities;
    
    ROS_INFO("Publishing strategy message");
    
    for (int i = 0; i < 6; i++)
    {
        velocities = strategy.getRobotVelocities(i);
        msg.lin_vel[i] = velocities[0];
        msg.ang_vel[i] = velocities[1];
        
        ROS_INFO("lin_vel: %f\t ang_vel: %f", msg.lin_vel[i], msg.ang_vel[i]);
    }
    
    publisher.publish(msg);
}

/**
 * Receives the robots locations from the vision topic.
 * @param msg a ROS string message pointer.
 */
void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg)
{
    ROS_INFO("Receiving vision message");
    
    for (int i = 0; i < 6; i++)
    {
        ROS_INFO("%d x: %f\t y: %f\t th: %f", i, msg->x[i], msg->y[i], msg->th[i]);
        strategy.setRobotPose(i, msg->x[i], msg->y[i], msg->th[i]);
    }
    
    strategy.setBallLocation(msg->ball_x, msg->ball_y);
}
