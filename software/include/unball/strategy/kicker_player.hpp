/**
 * @file   regular_player.hpp
 * @author Izabella Thais Oliveira Gomes
 * @date   22/10/2015
 *
 * @attention Copyright (C) 2015 UnBall Robot Soccer Team
 *
 * @brief
 */

#ifndef UNBALL_KICKER_PLAYER_H_
#define UNBALL_KICKER_PLAYER_H_

#include <unball/strategy/player.hpp>
#include <unball/strategy/goals.hpp>

class KickerPlayer : public Player
{
  public:
  	void buildPotentialFields(int robot_number);
  	bool isInBallRange(int robot_number);

  private:
  	static float const BALL_RANGE_ = 0.5;
};

#endif  // UNBALL_KICKER_PLAYER_H_