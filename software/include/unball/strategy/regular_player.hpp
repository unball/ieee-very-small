/**
 * @file   regular_player.hpp
 * @author Icaro da Costa Mota
 * @date   03/09/2015
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
  	void buildPotentialFields(int robot_number);
  	bool isInBallRange(int robot_number);
};

#endif  // UNBALL_REGULAR_PLAYER_H_