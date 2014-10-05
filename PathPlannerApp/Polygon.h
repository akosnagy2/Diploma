#pragma once
#include <vector>
#include "Point.h"

using namespace std;

struct Polygon
{
	Polygon()
	{}
	void AddPoint(Point p)
	{
		ps.push_back(p);
	}

	vector<Point> ps;
};