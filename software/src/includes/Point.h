/*
 * Point.h
 *
 *  Created on: Mar 28, 2014
 *      Author: icaro
 *
 *	Description: Expresses a position of a point in 2D
 *
 */

#ifndef POINT_H_
#define POINT_H_

#include <iostream>
#include "math.h"

class Point{
  public:
	/*Methods for setting and getting variables*/
	Point();
	Point(float x, float y);
	void Set(Point point);
	void Set(float x, float y);
	float GetX() {return(x);}
	float GetY() {return(y);}

	/*Operations*/
	Point Add(Point other_point);
	Point Add(float x, float y);
	Point Subtract(Point other_point);
	Point Multiply(float k);
	Point Divide(float k);
	void Move(float distance, float angle);
	void MoveXY(float x, float y);

	/*Getting information on two points*/
	bool IsOnTheLeftOf(Point other_point);
	bool IsOnTheRightOf(Point other_point);
	bool HasSameX(Point other_point);
	bool IsAbove(Point other_point);
	bool IsLowerThan(Point other_point);
	bool HasSameY(Point other_point);
	bool IsEqualTo(Point other_point);
	float DistanceTo(Point other_point);
	float Inclination(Point other_point);
  private:
	float x,y;
};

#endif /* POINT_H_ */
