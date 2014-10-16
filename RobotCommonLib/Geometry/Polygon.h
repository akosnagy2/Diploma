#pragma once
#include <vector>
#include "Point.h"
#include "Config.h"

using namespace std;

namespace PathPlanner
{

	struct Polygon
	{
		Polygon()
		{}
		void AddPoint(Point p)
		{
			ps.push_back(p);
		}
		Polygon TransformToWorld(Config q);
		Polygon TransformToLocal(Config q);

		vector<Point> ps;
	};

}