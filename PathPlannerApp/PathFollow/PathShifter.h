#pragma once
#include <vector>
#include "CarLikeRobot.h"
#include "PathSegment.h"
#include "Geometry/Config.h"

class PathShifter
{
public:
	PathShifter(PathSegment &pathSegment, CarLikeRobot &car);
	PathSegment getShiftedPath(float predict);
	~PathShifter();
private:
	PathSegment &pathSegment;
	CarLikeRobot &car;

	Config transformPosition(Config &pos, float predict);
};

