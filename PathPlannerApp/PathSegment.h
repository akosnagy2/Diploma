#pragma once

#include <vector>
#include "Geometry\Config.h"

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