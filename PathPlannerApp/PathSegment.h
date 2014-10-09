#pragma once

#include <vector>
#include "PathPlannerApp\PathPlanner\Config.h"

using namespace std;

namespace PathPlanner
{
	struct PathSegment
	{
		vector<Config> path;
		vector<float> curvature;
		bool direction; //true -> forward
	};
}