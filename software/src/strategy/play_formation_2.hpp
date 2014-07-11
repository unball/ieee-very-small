/**
 * @file   play_formation_2.hpp
 * @author Matheus Vieira Portela
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Initial formation #2, consisting of both the robots in the defense.
 */

#ifndef UNBALL_PLAY_FORMATION_2_H_
#define UNBALL_PLAY_FORMATION_2_H_

#include "play.hpp"

#define INITIAL_PLAY_STATE 0

class PlayFormation2 : public Play
{
  public:
	PlayFormation2();
  private:
	void initialRosMessage();
	void setUnfinishedActions();
	bool act();
};

#endif  // UNBALL_PLAY_FORMATION_2_H_
