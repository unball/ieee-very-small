/**
 * @file   goalkeeper.hpp
 * @author Icaro da Costa Mota
 * @date   08/09/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_GOALKEEPER_H_
#define UNBALL_GOALKEEPER_H_

#include <unball/strategy/player.hpp>

class Goalkeeper : public Player
{
  public:
  	Goalkeeper();
  	void buildPotentialFields(int robot_number);
  private:
  	bool isBallBelowGoalkeeper(int goalkeeper_number);
  	void stayAtTheBoundary();
  	bool isBallInRange();
  	void goToTheBoundary(float limit);

  	void updateBallPos();

  	Vector ball_pos_;

  	static float const LEFT_LIMIT = 0.18;
  	static float const RIGHT_LIMIT = -0.18;
};

#endif  // UNBALL_GOALKEEPER_H_