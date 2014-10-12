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
{}

float CarLineFollower::getFi(float v, float delta, float p, float predictLength)
{
	if(v < EPS)
		v = EPS;
	float L = predictLength;
	float kDelta = -0.9721f;//(c*L*L - b*L*v) / (v*v);
	float kP = -1.1662f;//-c*L / (v*v);

	float fi = delta * kDelta + p / 1000 * kP;

	/* Telítés értelmetlen értékekre */
	if(fi > M_PI_2 - EPS)
		fi = M_PI_2 - EPS;
	else if(fi < -M_PI_2 + EPS)
		fi = -M_PI_2 + EPS;

	return atan(robot.getAxisDistance() / L * tan(fi));
	//return fi;
}