/*
 * Point.h
 *
 *  Created on: Mar 28, 2014
 *      Author: icaro
 *
 *	Description: Expresses a position of a point in 2D
 *
 */

/**
 * @file   point.hpp
 * @author Icaro da Costa Mota
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief point class
 *
 * Defines a point, so you can deal with variables with 2 dimensions
 */

#ifndef UNBALL_POINT_H_
#define UNBALL_POINT_H_

#include <iostream>
#include "math.h"

class Point
{
  public:
	//Methods for setting and getting variables
	Point();
	Point(float x, float y);
	void Set(Point point);
	void Set(float x, float y);
	float GetX() {return(x);}
	float GetY() {return(y);}

	//Operations
	void Move(float distance, float angle);
	void MoveXY(float x, float y);
	Point operator+(Point& rhs) 
    Point operator-(Point& rhs) 
    Point operator*(float rhs) 
    Point operator/(float rhs)

	//Getting information on two points, considering a cartesian space
	bool IsOnTheLeftOf(Point other_point);
	bool IsOnTheRightOf(Point other_point);
	bool HasSameX(Point other_point);
	bool IsAbove(Point other_point);
	bool IsLowerThan(Point other_point);
	bool HasSameY(Point other_point);
	bool IsEqualTo(Point other_point);
	//TODO(mota.icaro@gmail.com):before doing this method, make sure you are dealing with a cartesian space. 
	//If not, change the above methods
	float Inclination(Point other_point);
	float DistanceTo(Point other_point);
	
  private:
	float x,y;
};

#endif // UNBALL_POINT_H_
