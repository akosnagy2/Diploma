#pragma once
class CarSpeedController
{
public:
	CarSpeedController(float P, float I, float D, float sampleT);
	~CarSpeedController();

	float getVelocity(float distError, float nextDist);

	float getSumError() { return sumError; }
	float getDistError() { return distError; }
private:
	float P;
	float I;
	float D;
	float sampleT;

	float distError;
	float sumError;
	float prevVel;
};

