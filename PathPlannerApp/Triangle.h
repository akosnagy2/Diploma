#pragma once
#include "Point.h"

struct Triangle
{
	Triangle(Point a, Point b, Point c)
	{
		this->p[0] = a;
		this->p[1] = b;
		this->p[2] = c;
	}
	Triangle()
	{}
	bool EdgeIntersect(Triangle &b, Point &i1, Point &i2)
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
	Point p[3];
};