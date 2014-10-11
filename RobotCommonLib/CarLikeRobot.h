#pragma once
#include "Position.h"
#define EPS 0.0001f
class CarLikeRobot
{
public:
	CarLikeRobot();
	~CarLikeRobot();

	/* Getters */
	float getAxisDistance() { return axisDistance; }
	float getFiMax() { return fiMax; }
	float getWheelDistance() { return wheelDistance; }
	float getWheelDiameter() { return wheelDiameter; }

	Position getPosition();

	/* Setters */
	void setAxisDistance(float axisD) { axisDistance = axisD; }
	void setFiMax(float fiMax) { this->fiMax = fiMax; }
	void setWheelDistance(float wDist) { wheelDistance = wDist; }
	void setWheelDiameter(float wDiam) { wheelDiameter = wDiam; }
	void setParameters(float motoraMax, float motorvMax, float motorSmoothFactor,
		float motorMultFactor, float steervMax, float timeStep)
	{
		this->motoraMax = motoraMax;
		this->motorvMax = motorvMax;
		this->motorSmoothFactor = motorSmoothFactor;
		this->motorMultFactor = motorMultFactor;
		this->steervMax = steervMax;
		this->timeStep = timeStep;
	}

	/* Model */
	void modelRobot(float v, float fi);

	/* Calculators */
	float getMaxRadiusRatio(float kappa);
	

private:
	/* Phisical parameters */
	float axisDistance;
	float wheelDistance;
	float wheelDiameter;

	/* Kinematical parameters */
	float fiMax;
	float motoraMax;
	float motorvMax;
	float motorSmoothFactor;
	float motorMultFactor;
	float steervMax;

	/* Misc. paremeters */
	float timeStep;

	/* State variables */
	float robotSpeed;
	float robotSteer;
	Position position;
};

