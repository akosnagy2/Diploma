#pragma once

#include <vector>
#include "Geometry\Config.h"

using namespace std;

#define POINT_SEARCH_LIMIT	10

namespace PathPlanner
{
	struct PathSegment
	{
		vector<Config> path;
		vector<float> velocity;
		vector<float> curvature;
		bool direction; //true -> forward

		int getClosestPoint(Config point, int start);
	};
}