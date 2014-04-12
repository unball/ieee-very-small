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

void publishVel(ros::Publisher &publisher);

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator_node");
   
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
    while (ros::ok())
    {
        publishVel(publisher);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

void publishVel(ros::Publisher &publisher)
{
    geometry_msgs::Twist msg;
    
    ROS_DEBUG("Publishing cmd vel");
    
    msg.linear.x = 0.10; // Set to make the robot turn
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.5; // Set to make the robot move
    
    publisher.publish(msg);
}
