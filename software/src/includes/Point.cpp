/*
 * Point.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: icaro
 */

#include "Point.h"

Point::Point(){
	x=0, y=0;
}

Point::Point(float x, float y){
	this->x=x, this->y=y;
}

void Point::Set(Point point){
	x=point.GetX(); y=point.GetY();
}

void Point::Set(float x, float y){
	this->x=x,this->y=y;
}

Point operator+(Point& rhs) {
    return (Point(x + rhs.x,y + rhs.y));
}

Point operator-(Point& rhs) {
    return (Point(x - rhs.x,y - rhs.y));
}

Point operator*(float rhs) {
    return (Point(x*rhs,y*rhs));
}

Point operator/(float rhs) { //Look out, do not divide by 0
    return (Point(x/rhs,y/rhs));
}

float Point::DistanceTo(Point other_point){
	float x_distance = x - other_point.GetX();
	float y_distance = y - other_point.GetY();
	return(sqrt(pow(x_distance,2) + pow(y_distance,2)));
}

bool Point::IsOnTheLeftOf(Point other_point){
	return ((this->x < other_point.GetX()) ?  true : false);
}
bool Point::IsOnTheRightOf(Point other_point){
	return ((this->x > other_point.GetX()) ?  true : false);
}
bool Point::HasSameX(Point other_point){
	return ((this->x == other_point.GetX()) ?  true : false);
}
bool Point::IsAbove(Point other_point){
    return(this->y > other_point.GetY() ? true:false);
}
bool Point::IsLowerThan(){
    return(this->y < other_point.GetY() ? true:false);
}
bool Point::HasSameY(Point other_point){
	return ((this->y == other_point.GetY()) ?  true : false);
}

bool Point::IsEqualTo(Point other_point){
	return (HasSameX(other_point) && HasSameY(other_point));
}

void Point::Move(float distance, float angle){
	x += (distance*cos(angle));
	y += (distance*sin(angle));
}

void Point::MoveXY(float x, float y){
	this->x += x;
	this->y += y;
}
