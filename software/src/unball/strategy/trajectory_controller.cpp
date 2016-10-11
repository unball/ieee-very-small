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
    player_[0] = new AssistentPlayer();
    player_[1] = new AssistentPlayer();
    player_[2] = new Goalkeeper();
    direct_motion_ = true;
}

TrajectoryController::~TrajectoryController()
{
    for (int i = 2; i >= 0; --i)
        delete player_[i];
}

void TrajectoryController::run()
{
    Vector resultant_force;
    for (int i=0;i<3;i++)
    {
        player_[i]->buildPotentialFields(i);
        resultant_force = player_[i]->calculateResultantForce(i);
        //ROS_ERROR("resultant_force: mag: %.2f ang: %.2f", resultant_force.getMagnitude(),
            resultant_force.getDirection()*180/M_PI);
        player_[i]->clearPotentialFields();
        controlRobot(i, resultant_force);
    }
    robot[2].setAngVel(5);
    robot[2].setLinVel(0);
}

void TrajectoryController::controlRobot(int robot_number, Vector force)
{
    // Use histeresis to void quickly changing direction.
    float error_margin = 15*M_PI/180;

    if (direct_motion_)
    {
        if (fabs(math::reduceAngle(M_PI + force.getDirection() - robot[robot_number].getTh()))
            > M_PI/2 + error_margin)
        {
            direct_motion_ = false;
        }
    }
    else
    {
        if (fabs(math::reduceAngle(M_PI + force.getDirection() - robot[robot_number].getTh()))
            < M_PI/2 - error_margin)
        {
            direct_motion_ = true;
        }
    }

    //HACK: issue80 -  We have to add M_PI because the force is being calculated backwards
     if (direct_motion_) 
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

    // if (fabs(distance) > distance_tolerance)
        robot[robot_number].setLinVel(lin_vel);
    // else
    //     robot[robot_number].setLinVel(0);
}

void TrajectoryController::turn(int robot_number, float angle)
{
    const float ANG_KP = 0.008;
    const float ANG_KD = 0;

    //HACK: issue80 - Should be: angle - robot.getTh()
    float angle_error = math::reduceAngle(robot[robot_number].getTh() - angle);
    float angle_error_d = angle_error - angle_error_prev_;
    float ang_vel = ANG_KP*angle_error + ANG_KD*angle_error_d; // PD control
    angle_error_prev_ = angle_error;

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
    if (behaviour == KICKER_PLAYER)
        player_[robot_number] = new KickerPlayer();
    if (behaviour == ASSISTENT_PLAYER)
        player_[robot_number] = new AssistentPlayer();
    if (behaviour == GOALKEEPER_KICKER)
        player_[robot_number] = new GoalkeeperKicker();
}

Player* TrajectoryController::getPlayer(int robot_number)
{
    return player_[robot_number];
}
