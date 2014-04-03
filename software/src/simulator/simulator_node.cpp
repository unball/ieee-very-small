/**
 * @file   simulator_node.cpp
 * @author Icaro da Costa Mota
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Has a Gazebo plugin for controlling the robots and publish the state of the simulation
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void publishCmdVel(ros::Publisher &publisher);

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator_node");
   
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
    while (ros::ok())
    {
        publishCmdVel(publisher);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

void publishCmdVel(ros::Publisher &publisher)
{
    geometry_msgs::Twist msg;
    
    ROS_DEBUG("Publishing cmd vel");
    
    msg.linear.x = 0.1;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    
    publisher.publish(msg);
}
