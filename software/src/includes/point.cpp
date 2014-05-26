/**
 * @file   point.cpp
 * @author Icaro da Costa Mota
 * @author Gabriel Naves da Silva
 * @date   13/05/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Point class
 *
 * Implements geometry methods for dealing with points in 2D
 */

#include "point.hpp"

// Methods for setting and getting variables
Point::Point() { x = 0, y = 0; }
Point::Point(float x, float y) {this->x = x, this->y = y; }
void Point::Set(const Point& point) { x = point.x, y = point.y; }
void Point::Set(float x, float y) { this->x = x, this->y = y; }
float Point::GetX() { return x; }
float Point::GetY() { return y; }

// Operators
Point Point::operator+(const Point& point) { return Point(x+point.x, y+point.y); }
Point Point::operator-(const Point& point) { return Point(x-point.x, y-point.y); }
Point Point::operator*(float rhs) { return Point(x*rhs, y*rhs); }
Point Point::operator/(float rhs)
{
    if (rhs == 0) return Point();
    return Point(x/rhs, y/rhs);
}


float Point::DistanceTo(Point other_point)
{
	float x_distance = x - other_point.GetX();
	float y_distance = y - other_point.GetY();
	return(sqrt(pow(x_distance,2) + pow(y_distance,2)));
}

bool Point::IsOnTheLeftOf(Point other_point)
{
	return ((this->x < other_point.GetX()) ?  true : false);
}

bool Point::IsOnTheRightOf(Point other_point)
{
	return ((this->x > other_point.GetX()) ?  true : false);
}

bool Point::HasSameX(Point other_point)
{
	return ((this->x == other_point.GetX()) ?  true : false);
}

bool Point::IsAbove(Point other_point)
{
    return(this->y > other_point.GetY() ? true:false);
}

bool Point::IsLowerThan(Point other_point)
{
    return(this->y < other_point.GetY() ? true:false);
}

bool Point::HasSameY(Point other_point)
{
	return ((this->y == other_point.GetY()) ?  true : false);
}

bool Point::IsEqualTo(Point other_point)
{
	return (HasSameX(other_point) && HasSameY(other_point));
}

/**
 * Moves a point in a given angle from it (mesured from the x axis) in a given distance.
 * @distance the absolute value of the distance the point will travel. 
 * @angle the angle in which the distance must be traveled, in radians.
 * Note that the parameter may be negative,
 * which will result on an alteration of 180 degrees on the given angle
 */
void Point::Move(float distance, float angle)
{
	x += (distance*cos(angle));
	y += (distance*sin(angle));
}

