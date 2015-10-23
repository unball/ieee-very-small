/**
 * @file   trajectory_controller.cpp
 * @author Matheus Vieira Portela
 * @date   03/08/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief Control robots trajectory by applying potential fields.
 */

#include <unball/strategy/trajectory_controller.hpp>

TrajectoryController::TrajectoryController()
{
    player_[0] = new RegularPlayer();
    player_[1] = new RegularPlayer();
    player_[2] = new InitialGoalkeeper();
}

TrajectoryController::~TrajectoryController()
{
    for (int i = 2; i >= 0; --i)
        delete player_[i];
}

void TrajectoryController::run()
{
    Vector resultant_force;
    for (int i = 0; i < 2; ++i)
    {
        if (i==1) i = 2; //REMOVE THIS LINE, IT IS MEANT ONLY FOR TESTING. DO NOT FEAR IT
        player_[i]->buildPotentialFields(i);
        resultant_force = player_[i]->calculateResultantForce(i);
        player_[i]->clearPotentialFields();
        controlRobot(i, resultant_force);
    }
}

void TrajectoryController::controlRobot(int robot_number, Vector force)
{   
    if (fabs(math::reduceAngle(force.getDirection() - robot[robot_number].getTh())) <= M_PI/2) 
    {
        move(robot_number, force.getMagnitude());
        turn(robot_number, force.getDirection());
    }
    else
    {
        move(robot_number, -force.getMagnitude());
        turn(robot_number, math::invertAngle(force.getDirection()));
    }
}

void TrajectoryController::move(int robot_number, float distance)
{
    const float DIST_KP = 1;
    const float distance_tolerance = 0.05; // m

    float lin_vel = DIST_KP*distance; // P control

    if (fabs(distance) > distance_tolerance)
        robot[robot_number].setLinVel(lin_vel);
    else
        robot[robot_number].setLinVel(0);
}

void TrajectoryController::turn(int robot_number, float angle)
{
    const float ANG_KP = 0.03;
    const float ANG_KD = 0.003;

    float angle_error = math::reduceAngle(robot[robot_number].getTh() - angle);
    float ang_vel = ANG_KP*(angle_error + (angle_error - ANG_KD*angle_error_d_)); // PD control
    angle_error_d_ = angle_error;

    robot[robot_number].setAngVel(ang_vel);
}

void TrajectoryController::stopRobot(int robot_number)
{
    robot[robot_number].setAngVel(0);
    robot[robot_number].setLinVel(0);
}

void TrajectoryController::updatePlayer(int robot_number, player_behaviour behaviour)
{
    delete player_[robot_number];
    
    if (behaviour == INITIAL_GOALKEEPER)
        player_[robot_number] = new InitialGoalkeeper();
    if (behaviour == GOALKEEPER)
        player_[robot_number] = new Goalkeeper();
    if (behaviour == REGULAR_PLAYER)
        player_[robot_number] = new RegularPlayer();    
}

Player* TrajectoryController::getPlayer(int robot_number)
{
    return player_[robot_number];
}