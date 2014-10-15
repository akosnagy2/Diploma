#pragma once
#include "Point.h"
#include "Angle.h"

namespace PathPlanner
{

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
		const Config operator/(const float div) const
		{
			return Config(this->p/div, this->phi/div);
		}
		static float Distance(Config &a, Config &b)
		{
			return Point::Distance(a.p, b.p);
		}
		float PointAngularDistance(const Point &p, bool fromConfToPoint);

		Point p;
		float phi;
	};

}