#pragma once
#include <vector>
#include "CarLikeRobot.h"
#include "CarLineFollower.h"
#include "PathSegment.h"
#include "Geometry/Config.h"

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
	CarPathController(std::vector<PathSegment> &paths, CarLikeRobot &car, CarLineFollower &lf, float predict);
	~CarPathController();

	void Loop(Config nextPos);
	float getV() { return v; }
	float getFi() { return fi; }
	Config getRabbit() { return predictPoint; }
	float getTrackError() { return trackError; }
private:
	std::vector<PathSegment> &paths;
	std::vector<PathSegment> frontPath;
	CarLineFollower &lineFollower;
	CarLikeRobot &car;
	float predict;
	int index;
	int pathIndex;

	float trackError = 0.0f;
	Config predictPoint;
	float v;
	float fi;
	RobotState state;
	bool status;

	Config getSensorCenter(Config car);
};

