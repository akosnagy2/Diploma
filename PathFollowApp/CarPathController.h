#pragma once
#include <vector>
#include "Position.h"
#include "CarLikeRobot.h"
#include "CarLineFollower.h"
#include "CarSpeedController.h"

#define TURN_PHI 1000.0f

typedef enum StateEnum {
	init = 0,
	pathFollow,
	preStop,
	turn
} RobotState;

class CarPathController
{
public:
	CarPathController(std::vector<Position> &path, CarLikeRobot &car, CarLineFollower &lf, CarSpeedController &sc, float predict);
	~CarPathController();

	void Loop(Position nextPos);
	float getV() { return v; }
	float getFi() { return fi; }
	Position getRabbit() { return frontPath[predictIndex]; }
private:
	std::vector<Position> &path;
	std::vector<Position> frontPath;
	CarLineFollower &lineFollower;
	CarSpeedController &speedController;
	CarLikeRobot &car;
	float predict;
	int index;
	int predictIndex;

	Position pos;
	float v;
	float fi;
	RobotState state;
	bool status;
	int count;

	static float getDistance(Position a, Position b);
	Position getSensorCenter(Position car);
};

