#include "CarLikeRobot.h"
#include <math.h>
#include <vector>
#include <algorithm>

CarLikeRobot::CarLikeRobot()
{
	axisDistance = 295.0f;
	wheelDiameter = 105.0f;
	wheelDistance = 265.0f;
	fiMax = 0.3167f;

	motoraMax = 100.0f;
	motorvMax = 500.0f;
	motorSmoothFactor = 1.0f;
	motorMultFactor = 1.0f;
	steervMax = 1000.0f;
	timeStep = 0.1f;

	robotSpeed = 0.0f;
	robotSteer = 0.0f;
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

void CarLikeRobot::modelRobot(float v, float fi)
{
	float prevSpeed = robotSpeed;

	/* Beállási idõ */
	robotSpeed += (v - robotSpeed)*motorSmoothFactor;
	robotSpeed *= motorMultFactor;

	/* Sebesség telítés */
	if(robotSpeed > motorvMax)
		robotSpeed = motorvMax;
	if(robotSpeed < -motorvMax)
		robotSpeed = -motorvMax;

	/* Gyorsulás telítés */
	float accel = (robotSpeed - prevSpeed) / timeStep;
	if(accel > motoraMax)
		robotSpeed = prevSpeed + motoraMax * timeStep;
	if(accel < -motoraMax)
		robotSpeed = prevSpeed - motoraMax * timeStep;

	/* Kormány sebesség telítés */
	float steerSpeed = (fi - robotSteer) / timeStep;
	if(steerSpeed > steervMax)
		robotSteer = robotSteer + steervMax * timeStep;
	else if(steerSpeed < -steervMax)
		robotSteer = robotSteer - steervMax * timeStep;
	else
		robotSteer = fi;

	/* Kormány telítés */
	if(robotSteer > fiMax)
		robotSteer = fiMax;
	else if(robotSteer < -fiMax)
		robotSteer = -fiMax;
}

Position CarLikeRobot::getPosition()
{
	position.phi += 0.5f * robotSpeed * tan(robotSteer) / axisDistance;
	position.x += robotSpeed * cos(position.phi);
	position.y += robotSpeed * sin(position.phi);
	position.phi += 0.5f * robotSpeed * tan(robotSteer) / axisDistance;
	return position;
}