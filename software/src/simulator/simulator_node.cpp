/**
 * @file   simulator_node.cpp
 * @author Icaro da Costa Mota
 * @author Matheus Vieira Portela
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Implements a simulator node with communication for controlling the robots velocities and publish their poses
 * in the vision topic, emulating the real usage in the strategy node perspective.
 */
 
#include <vector>
#include <cmath>
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>

#include "unball/StrategyMessage.h"
#include "unball/VisionMessage.h"

void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg);
void publishVelocities(ros::Publisher &publisher, float lin_vel, float ang_vel);
void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg);
float convertQuaternionToEuler(float x, float y, float z, float w);
void publishVisionMessage(ros::Publisher &publisher, std::vector<float> &x, std::vector<float> &y, std::vector<float> &th,
                          std::vector<float> &ball_location);

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
        ros::spinOnce();
        loop_rate.sleep();
    }    
    
    return(0);
}

/**
 * Receives the six robots velocities through the strategy topic and publishes the velocities to the Gazebo simulation.
 * @param msg a ROS string message pointer.
 */
void receiveStrategyMessage(const unball::StrategyMessage::ConstPtr &msg)
{
    int publisher_index;
    
    ROS_DEBUG("Receiving strategy message");
    
    for (int i = 0; i < 6; i++)
    {
        ROS_DEBUG("lin_vel: %f\t ang_vel: %f", msg->lin_vel[i], msg->ang_vel[i]);
        publishVelocities(publisher[i], msg->lin_vel[i], msg->ang_vel[i]);
    }
}

/**
 * Publishes a twist message for controlling linear and angular velocities of each robot in the proper Gazebo topic.
 * @param publisher a ROS message publisher pointer.
 * @param lin_vel the desired robot linear velocity.
 * @param ang_vel the desired robot angular velocity.
 */
void publishVelocities(ros::Publisher &publisher, float lin_vel, float ang_vel)
{
    geometry_msgs::Twist msg;
    
    ROS_DEBUG("Publishing cmd vel");
    
    msg.linear.x = ang_vel;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = lin_vel;
    
    publisher.publish(msg);
}

/**
 * Receives the simulation model states and publishes the robot poses to strategy. Ignores all message positions that
 * is not a robot information.
 * Since robot orientation is given as a quaternion, it first converts it to an Euler angle on the XY plane.
 * @param msg a Gazebo message pointer.
 */
void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int msg_size = msg->name.size();
    int robot_index;
    std::vector<float> x(6), y(6), th(6);
    std::vector<float> ball_location(2);
    
    ROS_DEBUG("Receiving Gazebo model states");
    
    for (int i = 0; i < msg_size; ++i)
    {
        if (msg->name[i] == "robot_1")
        {
            robot_index = 0;
        }
        else if (msg->name[i] == "robot_2")
        {
            robot_index = 1;
        }
        else if (msg->name[i] == "robot_3")
        {
            robot_index = 2;
        }
        else if (msg->name[i] == "robot_4")
        {
            robot_index = 3;
        }
        else if (msg->name[i] == "robot_5")
        {
            robot_index = 4;
        }
        else if (msg->name[i] == "robot_6")
        {
            robot_index = 5;
        }
        else if (msg->name[i] == "ball")
        {
            ball_location[0] = msg->pose[i].position.x;
            ball_location[1] = msg->pose[i].position.y;
            continue;
        }
        else
        {
            continue; // ignore message if it is not a robot
        }
        
        x[robot_index] = msg->pose[i].position.x;
        y[robot_index] = msg->pose[i].position.y;
        th[robot_index] = convertQuaternionToEuler(msg->pose[i].orientation.x, msg->pose[i].orientation.y,
                                                   msg->pose[i].orientation.z, msg->pose[i].orientation.w);
    }
    
    publishVisionMessage(poses_publisher, x, y, th, ball_location);
}

/**
 * Converts a quaternion to Euler angle on the XY plane (or around the Z-axis). Based on the approach available at:
 * http://stackoverflow.com/questions/14447338/converting-quaternions-to-euler-angles-problems-with-the-range-of-y-angle
 * @param x quaternion x.
 * @param y quaternion y.
 * @param z quaternion z.
 * @param w quaternion w.
 * @return th angle on the XY plane.
 */
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

/**
 * Publishes a vision message with the updated robot poses and ball location.
 * @param publisher a ROS message publisher pointer.
 * @param x robots x coordinate.
 * @param y robots y coordinate.
 * @param th robots orientation angle.
 * @param ball_location vector containing the ball (x, y) coordinates.
 */
void publishVisionMessage(ros::Publisher &publisher, std::vector<float> &x, std::vector<float> &y, std::vector<float> &th,
                          std::vector<float> &ball_location)
{
    unball::VisionMessage message;
 
    ROS_DEBUG("Publishing robots locations");
    for (int i = 0; i < 6; i++)
    {
        ROS_DEBUG("Robot %d: x: %f, y: %f, th: %f", i, x[i], y[i], th[i]);
        message.x[i] = x[i];
        message.y[i] = y[i];
        message.th[i] = th[i];
    }
    
    ROS_DEBUG("Ball: x: %f, y: %f", ball_location[0], ball_location[1]);
    message.ball_x = ball_location[0];
    message.ball_y = ball_location[1];
    
    publisher.publish(message);
}
