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
    player_[2] = new Goalkeeper();
}

TrajectoryController::~TrajectoryController()
{
    for (int i = 3; i >= 0; --i)
        delete player_[i];
}

void TrajectoryController::run()
{
    Vector resultant_force;
    for (int i = 0; i < 3; ++i)
    {
        player_[i]->buildPotentialFields(i);
        resultant_force = player_[i]->calculateResultantForce(i);
        player_[i]->clearPotentialFields();
        controlRobot(i, resultant_force);
    }
}

void TrajectoryController::controlRobot(int robot_number, Vector force)
{
    move(robot_number, force.getMagnitude());
    turn(robot_number, force.getDirection());
}

void TrajectoryController::stopRobot(int robot_number)
{
    robot[robot_number].setAngVel(0);
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

void TrajectoryController::move(int robot_number, float distance)
{
    const float DIST_KP = 1;
    const float distance_tolerance = 0.05; // m

    float lin_vel = DIST_KP*distance; // P control

    if (distance > distance_tolerance)
        robot[robot_number].setLinVel(lin_vel);
    else
        robot[robot_number].setLinVel(0);
}