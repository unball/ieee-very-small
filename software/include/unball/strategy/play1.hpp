/**
 * @file   play1.hpp
 * @author Matheus Vieira Portela
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Example play
 */

#ifndef UNBALL_PLAY1_H_
#define UNBALL_PLAY1_H_

#include "play.hpp"

#define INITIAL_PLAY_STATE 0

class Play1 : public Play
{
  public:
	Play1();
  private:
	void initialRosMessage();
	void setUnfinishedActions();
	bool act();
};

#endif  // UNBALL_PLAY1_H_
