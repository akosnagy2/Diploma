#pragma once

#include "Configuration.h"
#include "Shape.h"

class Robot
{
public:
	Robot();

	void setStart(Configuration s);
	void setGoal(Configuration g);
	void setBody(Shape b);
	void setAxleDistance(double L);
	void setWheelBase(double wb);
	void setWheelDiameter(double wd);
	void setWheelWidth(double ww);
	void setPhiMax(double phi);

	Configuration& getStart();
	Configuration& getGoal();
	Shape& getBody();
	double getAxleDistance();
	double getWheelBase();
	double getWheelDiameter();
	double getWheelWidth();
	double getPhiMax();
	double getRMin();
	double getMaxWidth();
private:
	Configuration start;
	Configuration goal;
	Shape body;
	double axleDistance;
	double wheelBase;
	double wheelDiameter;
	double wheelWidth;
	double phiMax;
	double rMin;
};

