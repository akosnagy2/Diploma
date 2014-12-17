#pragma once

#include "PathSegment.h"

typedef enum 
{
	arm = 0x00,
	localPlanner,
	alternativePlanner,
	solution,
	fail
} ccsStateTypedef;

bool CCSWrapper(PathPlanner::Scene &s, vector<PathPlanner::PathSegment> &path);