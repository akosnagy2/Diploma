#pragma once
#include "Point.h"

namespace PathPlanner
{


	struct Triangle
	{
		Triangle(Point a, Point b, Point c);
		Triangle();
		bool EdgeIntersect(Triangle &b, Point &i1, Point &i2);
		bool PointInside(Point a);

		Point p[3];
	};

}