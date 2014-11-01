/**
 * @file   strategy_node.cpp
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Run strategy for robots
 * 
 * This node reads and publishes keyboard input to "strategy_control_topic" without requiring the user to press ENTER.
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <unball/KeyboardMessage.h>

#define KEY_ESC 27

int getch();
void publishKey(ros::Publisher &publisher, char c);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_node");
    
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<unball::KeyboardMessage>("keyboard_topic", 1);
    char c;
    
    while (true)
    {
        if ((c = getch()) != KEY_ESC)
        {
            publishKey(publisher, c);
            ros::spinOnce();
        }
    }
    
    return 0;
}

/**
 * Read a char from standard input without blocking the execution of a program
 */
int getch()
{
    static struct termios oldt, newt;
    int c;

    // Save old settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable buffering
    newt.c_lflag &= ~(ICANON);
    //newt.c_lflag &= ~(ICANON|ECHO); // Disable echo to the screen

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read character (non-blocking)
    c = getchar();

    // Restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return c;
}

/**
 * Publishes the robots velocities to the strategy topic.
 * @param publisher a ROS node publisher.
 * @param c Key to be published.
 */
void publishKey(ros::Publisher &publisher, char c)
{
    unball::KeyboardMessage msg;
    
    msg.key = c;
    ROS_INFO("Publishing key: %c", c);
    
    publisher.publish(msg);
}
