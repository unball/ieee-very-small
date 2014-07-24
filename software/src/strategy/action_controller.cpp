/**
 * @file   action_controller.cpp
 * @author Matheus Vieira Portela
 * @date   25/04/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Action class
 *
 * Implements robots actions for strategy
 */

#include <unball/strategy/action_controller.hpp>

/**
 * Action controller global object
 */
ActionController action_controller;

/**
 * Execute an action with respect to the current motion state. Whenever an execution returns true, meaning that it has
 * completed its goal, changes the motion state to STOP.
 */
void ActionController::run()
{
    bool has_action_finished;
    
    for (int i = 0; i < 6; ++i)
    {
        ROS_DEBUG("Robot action state: %d", robot[i].getMotionState());
        
        // Update previous motion state
        robot[i].setPreviousMotionState(robot[i].getMotionState());
        
        switch (robot[i].getMotionState())
        {
            case STOP:
                has_action_finished = executeStop(i);
                break;
            case MOVE:
                has_action_finished = executeMove(i);
                break;
            case LOOK_AT:
                has_action_finished = executeLookAt(i);
                break;
            case GO_TO:
                has_action_finished = executeGoTo(i);
                break;
        }
        
        if (has_action_finished)
            robot[i].setMotionState(STOP);
    }
}

/**
 * Check whether a robot has finished its action, which is characterized by a change in the motion state.
 * @param robot_number Number of the robot to be checked.
 */
bool ActionController::hasRobotFinished(int robot_number)
{
    // TODO [matheus.v.portela@gmail.com]: Choose which method will be used as a finishing criteria by 10-jun-2014.
    return (robot[robot_number].getMotionState() == STOP);
    //return robot[robot_number].hasMotionStateChanged();
}

/**
 * Change robot locomotion state to STOP, if it is not yet in it.
 * @param robot_number Number of the robot to be controlled.
 */
void ActionController::stop(int robot_number)
{
    if (robot[robot_number].getMotionState() != STOP)
        robot[robot_number].setMotionState(STOP);
}

/**
 * Stop all robot movements.
 * @param robot_number Number of the robot to be controlled.
 * @return always true, since it always is successful in stopping the movements.
 */
bool ActionController::executeStop(int robot_number)
{
    robot[robot_number].setLinVel(0);
    robot[robot_number].setAngVel(0);
    return true;
}

/**
 * Move a specified distance. Change robot locomotion state to MOVE, if it is not yet in it, and initialize movement
 * variables.
 * @param robot_number Number of the robot to be controlled.
 * @param distance Desired distance for the robot to move.
 */
void ActionController::move(int robot_number, float distance)
{
    if (robot[robot_number].getMotionState() != MOVE)
    {
        move_distance_[robot_number] = distance;
        move_initial_x_[robot_number] = robot[robot_number].getX();
        move_initial_y_[robot_number] = robot[robot_number].getY();
        robot[robot_number].setMotionState(MOVE);
    }
}

/**
 * Calculates the travelled distance from the initial move position to the current position. Keeps moving while the
 * travelled distance is less than the allowed tolerance.
 * @param robot_number Number of the robot to be controlled.
 * @return true if it has moved the required distance, false otherwise.
 */
bool ActionController::executeMove(int robot_number)
{
    const float tolerance = 0.02; // 2 cm
    
    float travelled_distance = robot[robot_number].calculateDistance(move_initial_x_[robot_number],
                                                                     move_initial_y_[robot_number]);
    float error = fabs(move_distance_[robot_number]) - travelled_distance;
    float vel = (move_distance_[robot_number] >= 0) ? 1 : -1;
    
    if (error > tolerance)
    {
        robot[robot_number].setLinVel(vel);
        return false;
    }
    else
    {
        robot[robot_number].setLinVel(0); // force it to a stop (may be unnecessary)
        return true;
    }
}

/**
 * Turn the robot to face a (x, y) coordinate. Change robot locomotion state to LOOK_AT, if it is not yet in it, and
 * initialize the look at variables.
 * @param robot_number Number of the robot to be controlled.
 * @param x Desired x coordinate for the robot to turn.
 * @param y Desired y coordinate for the robot to turn.
 */
void ActionController::lookAt(int robot_number, float x, float y)
{
    if (robot[robot_number].getMotionState() != LOOK_AT)
    {
        look_at_x_[robot_number] = x;
        look_at_y_[robot_number] = y;
        robot[robot_number].setMotionState(LOOK_AT);
    }
}

/**
 * Calculates the necessary angle to look at the (x, y) coordinate, based on the robot's current position. Keeps moving
 * while the difference between the current angle and the target angle is less than the allowed tolerance.
 * @param robot_number Number of the robot to be controlled.
 * @return true if its has finished turning to the (x, y) coordinate, false otherwise.
 */
bool ActionController::executeLookAt(int robot_number)
{
    const float tolerance = 0.1; // ~ 5.72 degrees
    
    float target_angle = robot[robot_number].calculateAngle(look_at_x_[robot_number], look_at_y_[robot_number]);
    float error = robot[robot_number].reduceAngle(robot[robot_number].getTh() - target_angle);
    float vel = (error > 0) ? 0.05 : -0.05;
    
    if (fabs(error) > tolerance)
    {
        robot[robot_number].setAngVel(vel);
        return false;
    }
    else
    {
        robot[robot_number].setAngVel(0);
        return true;
    }
}

/**
 * Move the robot to a (x, y) coordinate. Change robot locomotion state to GO_TO, if it is not yet in it, and initialize
 * the go to variables.
 * @param robot_number Number of the robot to be controlled.
 * @param x Desired x coordinate for the robot to go.
 * @param y Desired y coordinate for the robot to go.
 */
void ActionController::goTo(int robot_number, float x, float y)
{
    if (robot[robot_number].getMotionState() != GO_TO)
    {
        go_to_x_[robot_number] = x;
        go_to_y_[robot_number] = y;
        go_to_error_dist_i_[robot_number] = 0.0;
        go_to_error_dist_d_[robot_number] = 0.0;
        go_to_error_ang_i_[robot_number] = 0.0;
        go_to_error_ang_d_[robot_number] = 0.0;
        robot[robot_number].setMotionState(GO_TO);
    }
}

/**
 * Implements a PID controller for linear and angular velocities.
 * @param robot_number Number of the robot to be controlled.
 * @return true if its has arrived at the (x, y) coordinate, false otherwise.
 */
bool ActionController::executeGoTo(int robot_number)
{
    // PID controller constants
    const float DIST_KP = 3.0;
    const float DIST_KD = 0.1;
    
    const float ANG_KP = 0.03;
    const float ANG_KD = 0.003;
    
    const float distance_tolerance = 0.05; // m
    
    float target_angle = robot[robot_number].calculateAngle(go_to_x_[robot_number], go_to_y_[robot_number]);
    float distance_error = robot[robot_number].calculateDistance(go_to_x_[robot_number], go_to_y_[robot_number]);
    float angle_error = robot[robot_number].reduceAngle(robot[robot_number].getTh() - target_angle);
    float ang_vel = ANG_KP*(angle_error + (angle_error - ANG_KD*go_to_error_ang_d_[robot_number])); // PD control
    float lin_vel = DIST_KP*distance_error + DIST_KD*go_to_error_dist_i_[robot_number]; // PD control
    
    // Distance I and D error
    go_to_error_dist_i_[robot_number] += distance_error;
    go_to_error_dist_d_[robot_number] = distance_error;
    
    // Angle I and D error
    go_to_error_ang_i_[robot_number] += angle_error;
    go_to_error_ang_d_[robot_number] = angle_error;

    if (distance_error > distance_tolerance)
    {
        robot[robot_number].setLinVel(lin_vel);
        robot[robot_number].setAngVel(ang_vel);
        return false;
    }
    else
    {
        robot[robot_number].setLinVel(0);
        robot[robot_number].setAngVel(0);
        return true;
    }
}
