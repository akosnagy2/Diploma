#pragma once
class CarSpeedController
{
public:
	CarSpeedController(float P, float I, float D, float sampleT);
	~CarSpeedController();

	float getVelocity(float distError, float nextDist);
private:
	float P;
	float I;
	float D;
	float sampleT;

	float sumError;
	float prevVel;
};

