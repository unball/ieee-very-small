/**
 * @file   regular_player.hpp
 * @author Icaro da Costa Mota
 * @date   08/09/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_REGULAR_PLAYER_H_
#define UNBALL_REGULAR_PLAYER_H_

#include <unball/strategy/player.hpp>

class RegularPlayer : public Player
{
  public:
  	RegularPlayer();
  	void buildPotentialFields(int robot_number);
  	bool isInBallRange(int robot_number);

  private:
  	static float const BALL_RANGE_ = 0.5;
};

#endif  // UNBALL_REGULAR_PLAYER_H_