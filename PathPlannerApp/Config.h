#pragma once
#include "Point.h"
#include "path_planner_funcs.h"

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
		Config a(this->p + b.p, corrigateAngle(this->phi + b.phi));
		return a;
	}
	const Config operator-(const Config &b) const
	{
		Config a(this->p - b.p, corrigateAngle(this->phi - b.phi));
		return a;
	}
	static float Distance(Config &a, Config &b)
	{
		return Point::Distance(a.p, b.p);
	}
	float PointAngularDistance(const Point &p, bool fromConfToPoint)
	{
		float &theta_start = this->phi;
		float theta_end;

		if (fromConfToPoint)
			theta_end = atan2f(p.y - this->p.y, p.x - this->p.x);
		else
			theta_end = atan2f(this->p.y - p.y, this->p.x - p.x);

		float angDist1 = directedAngleDist(theta_start, theta_end, true);
		float angDist2 = directedAngleDist(theta_start, theta_end, false);

		if (fabs(angDist1) <  fabs(angDist2))
			return angDist1;
		else
			return angDist2;
	}
	Point p;
	float phi;
};

