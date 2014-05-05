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

Point Point::Add(Point other_point){
	return (Point(x+other_point.GetX(),y+other_point.GetY()));
}

Point Point::Add(float x, float y){
	return(Point(this->x+x,this->y+y));
}

Point Point::Subtract(Point other_point){
	return (Point(x-other_point.GetX(),y-other_point.GetY()));
}

Point Point::Multiply(float k){
	return(Point(k*x,k*y));
}

Point Point::Divide(float k){ //TODO: Apply exception in cases of division by 0
	if (k==0){
		std::cerr << "Cannot perform division by zero. Abort the program" << std::endl;
		exit(1);
	}
	return(Point(x/k,y/k));
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
