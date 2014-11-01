/**
 * @file   play.cpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Play class
 */

#include <unball/strategy/play.hpp>

Play::Play()
{
    /**
     * robots_action_finished_ is true when the last action has finished. Therefore, it must be initialized with false
     * indicating that no action was completed yet.
     */
    for (int i = 0; i < 6; ++i)
        robots_action_finished_[i] = false;
    for (int i = 0; i < 6; i++)
         play_state_[i] = INITIAL_PLAY_STATE;
}

std::string Play::getPlayName()
{
    return(play_name_);
}

void Play::initialRosMessage()
{
    std::string message = play_name_ + " RUN";
    ROS_INFO("%s", message.c_str());
}

/**
 * Implements the basic structure the plays must follow.
 */
bool Play::run()
{
    initialRosMessage();
    
    // finish the action of all robots that have ended their actions
    for (int i = 0; i < 6; ++i)	
        finishRobotAction(i);
    // for those who have not finished their actions yet, set action_finished_ to false
    setUnfinishedActions();
    return act(); //return true if action has finished
}

/**
 * Finish the action of the robot if the actionController says it should finish
 */
void Play::finishRobotAction(int i)
{
    if (action_controller.hasRobotFinished(i))
        robots_action_finished_[i] = true;
}