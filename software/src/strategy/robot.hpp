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

enum MotionState
{
    UNDEFINED,
    STOP,
    MOVE,
    LOOK_AT,
    GO_TO,
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
    MotionState getMotionState();
    
    void setX(float x);
    void setY(float y);
    void setTh(float th);
    void setLinVel(float lin_vel);
    void setAngVel(float ang_vel);
    void setMotionState(MotionState motion_state);
    bool hasMotionStateChanged();
    
    float saturate(float x, float limit);
    float reduceAngle(float angle);
    
    void run();
    void stop();
    bool executeStop();
    void move(float distance);
    bool executeMove();
    void lookAt(float x, float y);
    bool executeLookAt();
    void goTo(float x, float y);
    bool executeGoTo();
    
  private:
    // Location attributes
    float x_;
    float y_;
    float th_;
    
    // Velocity attributes
    float lin_vel_;
    float ang_vel_;
    
    // Motion attributes
    MotionState motion_state_;
    MotionState previous_motion_state_;
    
    // Move attributes
    float move_initial_x_;
    float move_initial_y_;
    float move_distance_;
    
    // Look at attributes
    float look_at_x_;
    float look_at_y_;
    
    // Go to attributes
    float go_to_x_;
    float go_to_y_;
    int go_to_state_;
};

#endif  // UNBALL_ROBOT_H_

