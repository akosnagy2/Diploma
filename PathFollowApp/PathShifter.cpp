#include "PathShifter.h"

PathShifter::PathShifter(std::vector<Position> &path, CarLikeRobot &car) :
path(path),
car(car)
{}


PathShifter::~PathShifter()
{}

std::vector<Position> PathShifter::getShiftedPath()
{
	std::vector<Position> shiftedPath(path.size());
	for(int i = 0; i < path.size(); i++) {
		shiftedPath[i] = transformPosition(path[i]);
	}
	return shiftedPath;
}

Position PathShifter::transformPosition(Position &pos)
{
	Position newPos;
	float sinF = sin(pos.phi);
	float cosF = cos( pos.phi);
	newPos.x = car.getAxisDistance() * cosF + pos.x;
	newPos.y = car.getAxisDistance() * sinF + pos.y;
	newPos.phi = pos.phi;
	return newPos;
}