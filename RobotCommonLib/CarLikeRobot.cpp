#include "CarLikeRobot.h"
#include <math.h>
#include <vector>
#include <algorithm>

#define EPS 0.0001

CarLikeRobot::CarLikeRobot()
{
	axisDistance = 295.0;
	wheelDiameter = 105.0;
	wheelDistance = 265.0;
	fiMax = 0.3167;
}


CarLikeRobot::~CarLikeRobot()
{
}

float CarLikeRobot::getMaxRadiusRatio(float kappa)
{
	kappa = abs(kappa);
	if (kappa < EPS)
		return 1.0f;

	float p = 1 + wheelDistance * 0.5f * kappa;
	return p / cos(atan(axisDistance / (1 / kappa + 0.5f * wheelDistance)));
}