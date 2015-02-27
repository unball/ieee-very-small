/**
 * @file   play_formation_2.hpp
 * @author Matheus Vieira Portela
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Initial formation #2, consisting of both the robots in the defense.
 */

#ifndef UNBALL_PLAY_FORMATION_2_H_
#define UNBALL_PLAY_FORMATION_2_H_

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/play.hpp>

class PlayFormation2 : public Play
{
  public:
    PlayFormation2();
    
  private:
  	void defineRobotNumbers();
    void act();
    void actState0(int robot);
    void actState1(int robot);
    void actState2(int robot);

    int left_defensive_robot_, right_defensive_robot_, neutral_robot_;
};

#endif  // UNBALL_PLAY_FORMATION_2_H_
