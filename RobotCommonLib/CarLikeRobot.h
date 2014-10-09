#pragma once
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

	/* Setters */
	void setAxisDistance(float axisD) { axisDistance = axisD; }
	void setFiMax(float fiMax) { this->fiMax = fiMax; }
	void setWheelDistance(float wDist) { wheelDistance = wDist; }
	void setWheelDiameter(float wDiam) { wheelDiameter = wDiam; }

	/* Calculators */
	float getMaxRadiusRatio(float kappa);

private:
	float axisDistance;
	float fiMax;
	float wheelDistance;
	float wheelDiameter;
};

