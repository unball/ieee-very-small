/**
 * @file   play_formation_1.hpp
 * @author Matheus Vieira Portela
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Initial formation #1, consisting of one robot in the attack and another in the defense.
 */

#ifndef UNBALL_PLAY_FORMATION_1_H_
#define UNBALL_PLAY_FORMATION_1_H_

#include <ros/ros.h>

#include <unball/strategy/action_controller.hpp>
#include <unball/strategy/play.hpp>

class PlayFormation1 : public Play
{
  public:
    PlayFormation1();
    
  private:
  	void defineRobotNumbers();
    void act();
    void actState0(int robot);
    void actState1(int robot);
    void actState2(int robot);

    int defensive_robot_, offensive_robot_, neutral_robot_;
};

#endif  // UNBALL_PLAY_FORMATION_1_H_
