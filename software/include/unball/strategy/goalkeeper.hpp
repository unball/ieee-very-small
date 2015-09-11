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
  	void buildPotentialFields(int robot_number);
};

#endif  // UNBALL_GOALKEEPER_H_