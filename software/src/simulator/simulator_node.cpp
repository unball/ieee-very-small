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
#include <std_msgs/String.h>

#include "simulation.hpp"

Simulator simulator;

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator_node");
   
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("strategy_topic", 1000, receiveStrategyMessage);
    ros::Publisher publisher = n.advertise<unball::SimulationMessage>("simulation_topic", 1000);
  
    while (ros::ok())
    {
        simulator.run();
        
        publishRobotsVelocities(publisher);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

publishRobotsVelocities(ros::Publisher &publisher)
{
    unball::StrategyMessage msg;
    std::vector<float> velocities;
    
    ROS_DEBUG("Publishing simulation message");
    
    for (int i = 0; i < 6; i++)
    {
        velocities = simulation.getRobotVelocities(i);
        msg.lin_vel[i] = velocities[0];
        msg.rot_vel[i] = velocities[1];
        
        ROS_DEBUG("lin_vel: %f\t rot_vel: %f", msg.lin_vel[i], msg.rot_vel[i]);
    }
    
    publisher.publish(msg);
}


void receiveStrategyMessage(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Receiving: [%s]", msg->data.c_str());

    for (int i = 0; i < msg->x.size(); i++)
    {
        ROS_DEBUG("%f", msg->x[i]);
        //simulator.SetRobotLocation();
        simulator.SetRobotLinearVel();
        simulator.SetRobotAngularVel();        
    }
}
