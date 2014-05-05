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

/*
float Point::Inclination(Point other_point){
	//Returns the inclination of the line given by two points (it may be negative)
	//remember that the lower point has the greatest y value
	if (IsEqualTo(other_point)) return 0;
	if (HasSameY(other_point))
		return(IsOnTheLeftOf(other_point) ? M_PI : 0);	//180 if it is on the left, 0 if it is on the right
	if (HasSameX(other_point))
		return(IsLowerThan(other_point) ? 3*M_PI/2 : M_PI/2);//270 degrees if is lower, 90 if it is above

	float theta = atan(fabs((y - other_point.GetY())/(x - other_point.GetX())));

	if (IsOnTheLeftOf(other_point))
		return(IsLowerThan(other_point) ? (2*M_PI)-theta : theta); //360-theta if lower, theta if above
	if (IsOnTheRightOf(other_point))
		return(IsLowerThan(other_point) ? M_PI + theta : M_PI - theta); //180 + theta if lower, 180 - theta if above
	return (0);
}
*/

bool Point::IsOnTheLeftOf(Point other_point){
	return ((this->x < other_point.GetX()) ?  true : false);
}
bool Point::IsOnTheRightOf(Point other_point){
	return ((this->x > other_point.GetX()) ?  true : false);
}
bool Point::HasSameX(Point other_point){
	return ((this->x == other_point.GetX()) ?  true : false);
}
/*bool Point::IsAbove(Point other_point){
	return ((this->y < other_point.GetY()) ?  true : false);
	//y grows as the image goes down. therefore, if y1<y2, y1 is above y2
}
bool Point::IsLowerThan(Point other_point){
	return ((this->y > other_point.GetY()) ?  true : false);
	//y grows as the image goes down. therefore, if y1>y2, y1 is lower than y2
}*/
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
