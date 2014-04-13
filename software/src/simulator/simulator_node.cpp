/**
 * @file   simulator_node.cpp
 * @author Icaro da Costa Mota
 * @author Matheus Vieira Portela
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Has a Gazebo plugin for controlling the robots and publish the state of the simulation
 */
 
#include <vector>
#include <cmath>
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>

#include "unball/StrategyMessage.h"
#include "unball/VisionMessage.h"

void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg);
void publishVelocities(ros::Publisher &publisher, float lin_vel, float rot_vel);
float convertQuaternionToEuler(float x, float y, float z, float w);
void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg);
void publishRobotsPoses(ros::Publisher &publisher, std::vector<float> &x, std::vector<float> &y, std::vector<float> &th);

ros::Publisher publisher[6], poses_publisher;

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator_node");
   
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber strategy_subcriber = n.subscribe("strategy_topic", 1, receiveStrategyMessage);
    ros::Subscriber gazebo_subcriber = n.subscribe("gazebo/model_states", 1, receiveGazeboModelStates);
    
    publisher[0] = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1);
    publisher[1] = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1);
    publisher[2] = n.advertise<geometry_msgs::Twist>("robot_3/cmd_vel", 1);
    publisher[3] = n.advertise<geometry_msgs::Twist>("robot_4/cmd_vel", 1);
    publisher[4] = n.advertise<geometry_msgs::Twist>("robot_5/cmd_vel", 1);
    publisher[5] = n.advertise<geometry_msgs::Twist>("robot_6/cmd_vel", 1);
    
    poses_publisher = n.advertise<unball::VisionMessage>("vision_topic", 1);
  
    while (ros::ok())
    {
        publishVelocities(publisher[0], 0.5, 0.1);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

/**
 * Receives the six robots velocities through the strategy topic.
 * 
 * @param msg a ROS string message pointer.
 */
void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg)
{
    int publisher_index;
    
    ROS_INFO("Receiving strategy message");
    
    for (int i = 0; i < 6; i++)
    {
        ROS_INFO("lin_vel: %f\t rot_vel: %f", msg->lin_vel[i], msg->rot_vel[i]);
        publishVelocities(publisher[i], msg->lin_vel[i], msg->rot_vel[i]);
    }
}

void publishVelocities(ros::Publisher &publisher, float lin_vel, float rot_vel)
{
    geometry_msgs::Twist msg;
    
    ROS_DEBUG("Publishing cmd vel");
    
    msg.linear.x = rot_vel; // Set to make the robot turn
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = lin_vel; // Set to make the robot move
    
    publisher.publish(msg);
}

// based on: http://stackoverflow.com/questions/14447338/converting-quaternions-to-euler-angles-problems-with-the-range-of-y-angle
float convertQuaternionToEuler(float x, float y, float z, float w)
{
    float th;
    double sqw = w*w;
    double sqx = x*x;
    double sqy = y*y;
    double sqz = z*z;
    
    th = (atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw)) * (180.0f/M_PI));
    
    return th;
}

void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    const int msg_size = msg->name.size();
    int robot_index;
    std::vector<float> x(6), y(6), th(6);
    
    for (int i = 0; i < msg_size; ++i)
    {
        if (msg->name[i] == "robot_1")
            robot_index = 0;
        else if (msg->name[i] == "robot_2")
            robot_index = 1;
        else if (msg->name[i] == "robot_3")
            robot_index = 2;
        else if (msg->name[i] == "robot_4")
            robot_index = 3;
        else if (msg->name[i] == "robot_5")
            robot_index = 4;
        else if (msg->name[i] == "robot_6")
            robot_index = 5;
        else
            continue; // ignore message if it is not a robot
        
        x[robot_index] = msg->pose[i].position.x;
        y[robot_index] = msg->pose[i].position.y;
        th[robot_index] = convertQuaternionToEuler(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);
    }
    
    publishRobotsPoses(poses_publisher, x, y, th);
}

void publishRobotsPoses(ros::Publisher &publisher, std::vector<float> &x, std::vector<float> &y, std::vector<float> &th)
{
    unball::VisionMessage message;
 
    ROS_INFO("Publishing robots locations");
    for (int i = 0; i < 6; i++)
    {
        ROS_INFO("Robot %d: x: %f, y: %f, th: %f", i, x[i], y[i], th[i]);
        message.x[i] = x[i];
        message.y[i] = y[i];
        message.th[i] = th[i];
    }
    
    publisher.publish(message);
}
