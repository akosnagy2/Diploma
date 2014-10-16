#include "Triangle.h"


Triangle::Triangle(Point& a, Point& b, Point& c)
{
	pointList p;
	p.push_back(a);
	p.push_back(b);
	p.push_back(c);
	this->setPoints(p);
}

Triangle::Triangle(pointList& pointArray)
{
	this->setPoints(pointArray);
}