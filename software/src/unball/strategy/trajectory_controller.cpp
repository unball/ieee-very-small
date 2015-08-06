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
}

TrajectoryController::~TrajectoryController()
{
    clearPotentialFields();
}

void TrajectoryController::run()
{
    buildPotentialFields();
    Vector resultant_force = calculateResultantForce(0);
    controlRobot(0, resultant_force);
    clearPotentialFields();

    ROS_INFO("Resultant force: %s", resultant_force.toString().c_str());
}

void TrajectoryController::buildPotentialFields()
{
    Vector ball_position(Vector(Ball::getInstance().getX(), Ball::getInstance().getY()));
    potential_fields_.push_back(new AttractivePotentialField(ball_position, 10));

    for (int i = 1; i < 6; ++i)
        potential_fields_.push_back(new RepulsivePotentialField(Vector(robot[i].getX(), robot[i].getY()), 3));
}

void TrajectoryController::clearPotentialFields()
{
    for (int i = 0; i < potential_fields_.size(); ++i)
        delete potential_fields_[i];

    potential_fields_.clear();
}

Vector TrajectoryController::calculateResultantForce(int robot_number)
{
    Vector resultant_force;
    Vector position(robot[robot_number].getX(), robot[robot_number].getY());

    for (int i = 0; i < potential_fields_.size(); ++i)
        resultant_force += potential_fields_[i]->calculateForce(position);

    return resultant_force;
}

void TrajectoryController::controlRobot(int robot_number, Vector force)
{
    move(robot_number, force.getMagnitude());
    turn(robot_number, force.getDirection());
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