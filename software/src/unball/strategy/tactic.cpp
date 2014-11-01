/**
 * @file   tactic.cpp
 * @author Icaro da Costa Mota
 * @date   20/09/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief tactic class
 */

#include <unball/strategy/tactic.hpp>

Tactic::Tactic(int robot_num)
{
    /**
     * robots_action_finished_ is true when the last action has finished. Therefore, it must be initialized with false
     * indicating that no action was completed yet.
     */
    robot_num_ = robot_num;
    robot_action_finished_ = false;
    
    tactic_state_ = INITIAL_TACTIC_STATE;
}

std::string Tactic::getTacticName()
{
    return(tactic_name_);
}

void Tactic::initialRosMessage()
{
    std::string message = tactic_name_ + " RUN";
    ROS_INFO("%s", message.c_str());
}

/**
 * Implements the basic structure the tactics must follow.
 */
bool Tactic::run()
{
    initialRosMessage();
    
    // finish the action of the robots if it has ended their actions
    finishRobotAction();
    // if it has not finished its action yet, set action_finished_ to false
    setUnfinishedActions();
    return act(); //return true if action has finished
}

/**
 * Finish the action of the robot if the actionController says it should finish
 */
void Tactic::finishRobotAction()
{
    if (action_controller.hasRobotFinished(robot_num_))
        robot_action_finished_ = true;
}