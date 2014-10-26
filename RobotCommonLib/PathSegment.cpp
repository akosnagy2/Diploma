#include "PathSegment.h"

using namespace PathPlanner;

int PathSegment::getClosestPoint(Config point, int start)
{
	int index = start - POINT_SEARCH_LIMIT, i;
	if(index < 0) {
		index = 0;
	}
	float min = Config::Distance(point, path[index]);
	float t;

	for(i = index + 1; i < std::min(start + POINT_SEARCH_LIMIT, (int)path.size()); i++) {
		t = Config::Distance(point, path[i]);
		if(t < min) {
			min = t;
			index = i;
		}
	}
	return index;
}