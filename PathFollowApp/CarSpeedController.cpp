#include "CarSpeedController.h"


CarSpeedController::CarSpeedController(float P, float I, float D, float sampleT) :
P(P),
I(I),
D(D),
sampleT(sampleT)
{
	sumError = 0;
	prevVel = 0;
}


CarSpeedController::~CarSpeedController()
{
}

float CarSpeedController::getVelocity(float distError, float nextDist)
{
	this->distError = distError + nextDist;
	sumError += distError;
	float s = nextDist + P * distError + I * sumError;
	float v = 2 * s / sampleT - prevVel;
	prevVel = v;
	return v;
}