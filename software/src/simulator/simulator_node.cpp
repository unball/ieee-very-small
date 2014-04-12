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
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

void publishVel(ros::Publisher &publisher, float lin_vel, float rot_vel);

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator_node");
   
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Publisher publisher_1 = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1);
    ros::Publisher publisher_2 = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1);
    ros::Publisher publisher_3 = n.advertise<geometry_msgs::Twist>("robot_3/cmd_vel", 1);
    ros::Publisher publisher_4 = n.advertise<geometry_msgs::Twist>("robot_4/cmd_vel", 1);
    ros::Publisher publisher_5 = n.advertise<geometry_msgs::Twist>("robot_5/cmd_vel", 1);
    ros::Publisher publisher_6 = n.advertise<geometry_msgs::Twist>("robot_6/cmd_vel", 1);
  
    while (ros::ok())
    {
        publishVel(publisher_1, 0.5, 0.1);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

void publishVel(ros::Publisher &publisher, float lin_vel, float rot_vel)
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
