#pragma once
#include <vector>
#include "Point.h"

using namespace std;

struct Polygon
{
	void AddPoint(Point p)
	{
		ps.push_back(p);
	}
	Polygon()
	{}
	vector<Point> ps;
};