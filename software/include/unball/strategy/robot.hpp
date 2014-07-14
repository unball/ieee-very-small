/**
 * @file   robot.hpp
 * @author Matheus Vieira Portela
 * @date   27/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Robot class
 *
 * Defines robots for strategy
 */

#ifndef UNBALL_ROBOT_H_
#define UNBALL_ROBOT_H_

#define ROBOT_SATURATION_LIN_VEL 5.0
#define ROBOT_SATURATION_ANG_VEL 2.0

#include "../geometry/point.hpp"

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

    Point getPos();
    float getX();
    float getY();
    float getTh();
    float getLinVel();
    float getAngVel();
    MotionState getMotionState();
    MotionState getPreviousMotionState();
    
    void setPosition(float x, float y);
    void setTh(float th);
    void setPose(float x, float y, float th);
    void setLinVel(float lin_vel);
    void setAngVel(float ang_vel);
    void setMotionState(MotionState motion_state);
    void setPreviousMotionState(MotionState previous_motion_state);
    bool hasMotionStateChanged();
    
    float saturate(float x, float limit);
    float reduceAngle(float angle);
    float calculateDistance(float x, float y);
    float calculateAngle(float x, float y);
    
  private:
    // Location attributes
    Point pos_;
    float th_;
    
    // Velocity attributes
    float lin_vel_;
    float ang_vel_;
    
    // Motion attributes
    MotionState motion_state_;
    MotionState previous_motion_state_;
};

extern Robot robot[6];

#endif  // UNBALL_ROBOT_H_

