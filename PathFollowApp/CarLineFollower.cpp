#include "CarLineFollower.h"

#define _USE_MATH_DEFINES
#include <math.h>

CarLineFollower::CarLineFollower(CarLikeRobot &robot, float w0, float ksi) :
robot(robot)
{
	b = 2 * w0 * ksi;
	c = w0 * w0 * ksi * ksi + w0 * w0 * (1 - ksi * ksi);
}


CarLineFollower::~CarLineFollower()
{
}

float CarLineFollower::getFi(float v, float delta, float p, float predictLength)
{
	if (v < EPS)
		v = EPS;
	float L = predictLength;
	float kDelta = (c*L*L - b*L*v) / (v*v);
	float kP = -c*L / (v*v);

	float fi = delta * kDelta + p * kP;

	/* Telítés értelmetlen értékekre */
	if (fi > M_PI - EPS)
		fi = M_PI - EPS;
	else if (fi < -M_PI + EPS)
		fi = -M_PI + EPS;
		
	return atan(robot.getAxisDistance() / L * tan(fi));
}