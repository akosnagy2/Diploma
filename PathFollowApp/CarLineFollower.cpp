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
	if(abs(v) < 20) // TODO ez se igaz�n j�, ink�bb a kappa �rt�keknek k�ne fix �rt�ket adni!
		return 0.0f;

	v = abs(v * 0.001f);

	float L = predictLength * 0.001f;
	float kDelta = (c*L*L - b*L*v) / (v*v);//-0.9721f;
	float kP = -c*L / (v*v);//-1.1662f;

	float fi = delta * kDelta + p / 1000 * kP;

	/* Tel�t�s �rtelmetlen �rt�kekre */
	if(fi > M_PI_2 - EPS)
		fi = M_PI_2 - EPS;
	else if(fi < -M_PI_2 + EPS)
		fi = -M_PI_2 + EPS;

	return atan(robot.getAxisDistance() * 0.001f / L * tan(fi));
}