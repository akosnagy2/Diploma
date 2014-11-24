#pragma once

#include <math.h>

namespace PathPlanner
{

	struct Point
	{
		Point()
		{}
		Point(float x, float y) 
		{
			this->x = x;
			this->y = y;
		}
		bool operator==(const Point &b) const
		{
			if ((this->x == b.x) && (this->y == b.y))
				return true;
			else
				return false;
		}
		const Point operator+(const Point &b) const
		{
			return Point(this->x + b.x, this->y + b.y);
		}
		const Point operator-(const Point &b) const
		{
			return Point(this->x - b.x, this->y - b.y);;
		}
		const Point operator*(const float mul) const
		{
			return Point(this->x*mul, this->y*mul);
		}
		const Point operator/(const float div) const
		{
			return Point(this->x/div, this->y/div);
		}
		static float Distance(const Point &a, const Point &b)
		{
			return sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
		}
		static float Distance2(const Point &a, const Point &b)
		{
			return ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
		}
		static float ScalarProduct(const Point &a, const Point &b)
		{
			return a.x*b.x + a.y*b.y;
		}
		static float atan2(const Point &a, const Point &b)
		{
			return atan2f(a.y - b.y, a.x - b.x);
		}

		float x;	// [mm]
		float y;	// [mm]
	};

}