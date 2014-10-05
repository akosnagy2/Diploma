#pragma once
#include "Point.h"
#include "Angle.h"

struct Config
{
	Config()
	{}
	Config(float x, float y, float phi)
	{
		p.x = x;
		p.y = y;
		this->phi = phi;
	}
	Config(Point p, float phi)
	{
		this->p = p;
		this->phi = phi;
	}
	bool operator==(const Config &b) const
	{
		if ((this->p == b.p) && (this->phi == b.phi))
			return true;
		else
			return false;
	}
	const Config operator+(const Config &b) const
	{
		return Config(this->p + b.p, Angle::Corrigate(this->phi + b.phi));
	}
	const Config operator-(const Config &b) const
	{
		return Config(this->p - b.p, Angle::Corrigate(this->phi - b.phi));
	}
	static float Distance(Config &a, Config &b)
	{
		return Point::Distance(a.p, b.p);
	}

	/*PointAngularDistance:
	* Signed smallest angular distance of a point and a configuration 
	*
	* The angular distance is measured assuming a preferred reference orientation, given by the input parameter 
	*   fromConfToPoint  -   positive: reference orientation points from the configuration position to the given point
	*						 negative: reference orientation points in the opposite direction
	*/
	float PointAngularDistance(const Point &p, bool fromConfToPoint)
	{
		float &theta_start = this->phi;
		float theta_end;

		if (fromConfToPoint)
			theta_end = atan2f(p.y - this->p.y, p.x - this->p.x);
		else
			theta_end = atan2f(this->p.y - p.y, this->p.x - p.x);

		float angDist1 = Angle::DirectedAngleDist(theta_start, theta_end, 1);
		float angDist2 = Angle::DirectedAngleDist(theta_start, theta_end, -1);

		if (fabs(angDist1) <  fabs(angDist2))
			return angDist1;
		else
			return angDist2;
	}

	Point p;
	float phi;
};

