#include "Triangle.h"

using namespace PathPlanner;

Triangle::Triangle()
{}

Triangle::Triangle(Point a, Point b, Point c)
{
	this->p[0] = a;
	this->p[1] = b;
	this->p[2] = c;
}

bool Triangle::EdgeIntersect(Triangle &b, Point &i1, Point &i2)
{
	bool p = false;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (this->p[i] == b.p[j])								
			{
				if (p)
				{
					i2 = this->p[i];
					return true;
				}
				else 
				{
					i1 = this->p[i];
					p = true;
				}
			}
		}
	}
	return false;
}

bool Triangle::PointInside(Point a)
{
	//http://www.blackpawn.com/texts/pointinpoly/
	
	// Compute vectors        
	Point v0 = p[2] - p[0];
	Point v1 = p[1] - p[0];
	Point v2 = a - p[0];

	// Compute dot products
	float dot00 = Point::ScalarProduct(v0, v0);
	float dot01 = Point::ScalarProduct(v0, v1);
	float dot02 = Point::ScalarProduct(v0, v2);
	float dot11 = Point::ScalarProduct(v1, v1);
	float dot12 = Point::ScalarProduct(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}