/**
 * @file   play_stop.hpp
 * @author Matheus Vieira Portela
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Stop play
 */

#ifndef UNBALL_PLAY_STOP_H_
#define UNBALL_PLAY_STOP_H_

#include "play.hpp"

class PlayStop : public Play
{
  public:
	PlayStop();
  private:
	void setUnfinishedActions();
	bool act();
};

#endif  // UNBALL_PLAY_STOP_H_
