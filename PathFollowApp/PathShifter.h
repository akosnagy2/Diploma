#pragma once
#include <vector>
#include "Position.h"
#include "CarLikeRobot.h"

class PathShifter
{
public:
	PathShifter(std::vector<Position>  &path, CarLikeRobot &car);
	std::vector<Position> getShiftedPath();
	~PathShifter();
private:
	std::vector<Position> &path;
	CarLikeRobot &car;

	Position transformPosition(Position &pos);
};

