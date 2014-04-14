/**
 * @file   robot.hpp
 * @author Matheus Vieira Portela
 * @date   27/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Robot class
 *
 * Defines robots for strategy
 */

#ifndef UNBALL_ROBOT_H_
#define UNBALL_ROBOT_H_

#define ROBOT_SATURATION_LIN_VEL 5.0
#define ROBOT_SATURATION_ANG_VEL 2.0

enum LocomotionState
{
    STOP,
    MOVE,
    TURN,
};

class Robot
{
  public:
    Robot();
    
    float getX();
    float getY();
    float getTh();
    float getLinVel();
    float getAngVel();
    LocomotionState getLocomotionState();
    
    void setX(float x);
    void setY(float y);
    void setTh(float th);
    void setLinVel(float lin_vel);
    void setAngVel(float ang_vel);
    void setLocomotionState(LocomotionState locomotion_state);
    
    float saturate(float x, float limit);
    
    void run();
    void stop();
    void executeStop();
    void move(float distance);
    void executeMove();
    void turn(float angle);
    void executeTurn();
    
  private:
    // Location attributes
    float x_;
    float y_;
    float th_;
    
    // Velocity attributes
    float lin_vel_;
    float ang_vel_;
    
    // Locomotion attributes
    LocomotionState locomotion_state_;
    
    // Move attributes
    float move_initial_x_;
    float move_initial_y_;
    float move_distance_;
    
    // Turn attributes
    float turn_initial_th_;
    float turn_angle_;
};

#endif  // UNBALL_ROBOT_H_

