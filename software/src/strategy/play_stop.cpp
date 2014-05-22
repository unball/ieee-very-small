#include "play_stop.hpp"
#include <ros/ros.h>
#include "action_controller.hpp" // ActionControler action_controller;

bool PlayStop::run()
{
    ROS_INFO("PLAY STOP");
    
    for (int i = 0; i < 6; ++i)
        action_controller.stop(i);
    
    return true;
}
