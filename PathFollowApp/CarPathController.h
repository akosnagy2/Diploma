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
	CarPathController(std::vector<Position> &path, CarLineFollower &lf, CarSpeedController &sc, int predict);
	~CarPathController();

	void Loop(Position nextPos);
	float getV() { return v; }
	float getFi() { return fi; }
private:
	std::vector<Position> &path;
	CarLineFollower &lineFollower;
	CarSpeedController &speedController;
	int predict;
	int index;

	Position pos;
	float v;
	float fi;
	RobotState state;
	bool status;
	int count;

	static float getDistance(Position a, Position b);
	static float wrapAngle(float phi);
	static Position getSensorCenter(Position car, Position point);
};

