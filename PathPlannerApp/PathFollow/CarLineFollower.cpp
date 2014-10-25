#include "CarLineFollower.h"

#define _USE_MATH_DEFINES
#include <math.h>

CarLineFollower::CarLineFollower(CarLikeRobot &robot, float w0, float ksi, float predictLength) : 
robot(robot)
{
	float b = 2 * w0 * ksi;
	float c = w0 * w0 * ksi * ksi + w0 * w0 * (1 - ksi * ksi);
	const float v = 0.5f;
	L = (predictLength + robot.getAxisDistance()) * 0.001f;
	kDelta = (c*L*L - b*L*v) / (v*v);
	kP = -c*L / (v*v);
}


CarLineFollower::~CarLineFollower()
{}

float CarLineFollower::getFi(float delta, float p)
{
	float fi = delta * kDelta + p * 0.001f * kP;

	/* Telítés értelmetlen értékekre */
	if(fi > M_PI_2 - EPS)
		fi = M_PI_2 - EPS;
	else if(fi < -M_PI_2 + EPS)
		fi = -M_PI_2 + EPS;

	return atan(robot.getAxisDistance() * 0.001f / L * tan(fi));
}

float CarLineFollower::getFi(float delta, float p, float predictDist)
{
	float fi = delta * kDelta + p * 0.001f * kP;

	/* Telítés értelmetlen értékekre */
	if(fi > M_PI_2 - EPS)
		fi = M_PI_2 - EPS;
	else if(fi < -M_PI_2 + EPS)
		fi = -M_PI_2 + EPS;

	return atan(robot.getAxisDistance() / predictDist * tan(fi));
}