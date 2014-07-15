#include <unball/strategy/play_stop.hpp>
#include <ros/ros.h>
#include <unball/strategy/action_controller.hpp> // ActionControler action_controller;

PlayStop::PlayStop()
{
    play_name_ = "PLAY STOP";
}

void PlayStop::setUnfinishedActions() {}

/**
 * Stops all actions from all robots
 */
bool PlayStop::act()
{
    for (int i = 0; i < 6; ++i)
        action_controller.stop(i);
    return true;
}
