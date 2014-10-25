#pragma once
#include "CarLikeRobot.h"
class CarLineFollower
{
public:
	CarLineFollower(CarLikeRobot &robot, float w0, float ksi, float predictLength);
	~CarLineFollower();

	float getFi(float delta, float p);
	float getFi(float delta, float p, float predictDist);
private:
	CarLikeRobot &robot;
	float L;
	float kDelta;
	float kP;
};

