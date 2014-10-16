/* 
 * File:   Circle.h
 * Author: Gábor
 *
 * Created on 2014. március 29., 14:13
 */

#ifndef CIRCLE_H
#define	CIRCLE_H

#include "Point.h"
#include "Line.h"


class Circle {
public:
    Circle(const Point& center, double radius);
    pointList getIntersections(const Line& line);
private:
    Point center;
    double radius;
};

#endif	/* CIRCLE_H */

