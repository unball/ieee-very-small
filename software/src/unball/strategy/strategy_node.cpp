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

#include <unball/MeasurementSystemMessage.h>
#include <unball/StrategyMessage.h>
#include <unball/KeyboardMessage.h>
#include <unball/strategy/strategy.hpp> // Strategy strategy;
#include <unball/strategy/robot.hpp> // Robot robot[6];
#include <unball/strategy/ball.hpp> // Ball ball;
#include <unball/strategy/goals.hpp> // Goals goals;

void initRobotsPoses();
void publishRobotsVelocities(ros::Publisher &publisher);
void receiveMeasurementSystemMessage(const unball::MeasurementSystemMessage::ConstPtr &msg);
void receiveKeyboardMessage(const unball::KeyboardMessage::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strategy_node");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(30); // Hz
    
    ros::Subscriber sub = n.subscribe("measurement_system_topic", 1, receiveMeasurementSystemMessage);
    ros::Subscriber sub2 = n.subscribe("keyboard_topic", 1, receiveKeyboardMessage);
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
        //msg.ang_vel[i] = robot[i].getAngVel();
        msg.ang_vel[i] = robot[i].getAngVel();

        ROS_DEBUG("lin_vel: %f\t ang_vel: %f", msg.lin_vel[i], msg.ang_vel[i]);
    }
    
    publisher.publish(msg);
}

float getAngularVel(int i) {
    float K = 1;
    float distance_x = Ball::getInstance().getX() - robot[i].getX();
    float distance_y = Ball::getInstance().getY() - robot[i].getY();
    float angle_to_ball = atan2(distance_y,distance_x);
    return K*(robot[i].getTh() - angle_to_ball);
}

/**
 * Receives the robots locations from the vision topic.
 * @param msg an UnBall vision message pointer.
 */
void receiveMeasurementSystemMessage(const unball::MeasurementSystemMessage::ConstPtr &msg)
{
    //ROS_INF)O("\n\n[StrategyNode]:ReceiveMeasuermentSystemMessage - Receiving measurement system message");
    
    for (int i = 0; i < 6; i++)
    {
        //ROS_INFO("Robots: %d x: %f\t y: %f\t th: %f", i, msg->x[i], msg->y[i], msg->th[i]);
        robot[i].setPose(msg->x[i], msg->y[i], msg->th[i]);
    }

    //ROS_INFO("Ball: x: %f, y: %f", msg->ball_x, msg->ball_y);
    if (msg->y[2] < -0.75 or msg->y[2] > 0.75 or msg->x[2] < -0.65 or msg->x[2] > 0.65) 
    {
        //HACK: in case we do not find our goakeeper, we set our goal by:
        //finding the opponent goalkeeper
        //sending the negative of its x position to the function setGoalkeeperSide
        //this way we find which is our goal, and therefore which goal we should target
        //only meant to happen on a penalty
        int opponent_goalkeeper_number = Goals::getInstance().findOpponentGoalkeeper();
        Goals::getInstance().setGoalkeeperSide(-robot[opponent_goalkeeper_number].getY());    
    }
    else 
    {
        Goals::getInstance().setGoalkeeperSide(robot[2].getY());
    }
    Ball::getInstance().update(msg->ball_y, -msg->ball_x);
}

/**
 * Receives a key from the keyboard topic.
 * @param msg a keyboard message pointer.
 */
void receiveKeyboardMessage(const unball::KeyboardMessage::ConstPtr &msg)
{
    ROS_ERROR("Received key: %c", msg->key);
    
    Strategy::getInstance().receiveKeyboardInput(msg->key);
    strategy.receiveKeyboardInput(msg->key);
}
