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
 * publishes to the strategy topic.
 */

#include <vector>

#include <ros/ros.h>

#include "unball/VisionMessage.h"
#include "unball/StrategyMessage.h"
#include "unball/StrategyControlMessage.h"
#include "strategy.hpp" // Strategy strategy;
#include "robot.hpp" // Robot robot[6];
#include "ball.hpp" // Ball ball;

void initRobotsPoses();
void publishRobotsVelocities(ros::Publisher &publisher);
void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg);
void receiveStrategyControlMessage(const unball::StrategyControlMessage::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(5); // Hz
    
    ros::Subscriber sub = n.subscribe("vision_topic", 1, receiveVisionMessage);
    ros::Subscriber sub2 = n.subscribe("strategy_control_topic", 1, receiveStrategyControlMessage);
    ros::Publisher publisher = n.advertise<unball::StrategyMessage>("strategy_topic", 1);
    
    initRobotsPoses();
    
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
 * Initialize robots poses with the simulation hardcoded poses.
 * 
 * This is done in order to prevent errors in the first evaluated action, which may calculate travelled distance from
 * the last iteration.
 * 
 * For example: if a robot is set to move for 0.20 m, its initial position is (0, 0) and it is spawn at (0.3, 0), the
 * method will calculate a travelled distance of 0.30 m and return, even though its real travelled distance so far is 0!
 */
void initRobotsPoses()
{
    float x[6] = {0.37, 0.37, 0.60, -0.37, -0.37, -0.60};
    float y[6] = {0.40, -0.40, 0.0, 0.40, -0.40, 0.0};
    
    for (int i = 0; i < 6; ++i)
        robot[i].setPose(x[i], y[i], 0.0); // Initial theta is 0
}

/**
 * Publishes the robots velocities to the strategy topic.
 * @param publisher a ROS node publisher.
 */
void publishRobotsVelocities(ros::Publisher &publisher)
{
    unball::StrategyMessage msg;
    
    ROS_DEBUG("Publishing strategy message");
    
    for (int i = 0; i < 6; i++)
    {
        msg.lin_vel[i] = robot[i].getLinVel();
        msg.ang_vel[i] = robot[i].getAngVel();
        
        ROS_DEBUG("lin_vel: %f\t ang_vel: %f", msg.lin_vel[i], msg.ang_vel[i]);
    }
    
    publisher.publish(msg);
}

/**
 * Receives the robots locations from the vision topic.
 * @param msg an UnBall vision message pointer.
 */
void receiveVisionMessage(const unball::VisionMessage::ConstPtr &msg)
{
    ROS_DEBUG("Receiving vision message");
    
    for (int i = 0; i < 6; i++)
    {
        ROS_DEBUG("%d x: %f\t y: %f\t th: %f", i, msg->x[i], msg->y[i], msg->th[i]);
        robot[i].setPose(msg->x[i], msg->y[i], msg->th[i]);
    }
    
    ball.update(msg->ball_x, msg->ball_y);
}

/**
 * Receives the strategy control message from the keyboard node.
 * @param msg an UnBall strategy control message message pointer.
 */
void receiveStrategyControlMessage(const unball::StrategyControlMessage::ConstPtr &msg)
{
    ROS_INFO("Received key: %c", msg->key);
}
