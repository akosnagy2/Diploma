#pragma once
#include "Shape.h"
class Triangle :
	public Shape
{
public:
	Triangle(Point& a, Point& b, Point& c);
	Triangle(pointList& pointArray);
};

