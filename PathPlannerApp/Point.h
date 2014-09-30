#pragma once

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
		Point a(this->x + b.x, this->y + b.y);
		return a;
	}
	const Point operator-(const Point &b) const
	{
		Point a(this->x - b.x, this->y - b.y);
		return a;
	}
	const Point operator*(const float mul) const
	{
		Point a(this->x*mul, this->y*mul);
		return a;
	}
	const Point operator/(const float div) const
	{
		Point a(this->x/div, this->y/div);
		return a;
	}
	static float Distance(Point &a, Point &b)
	{
		return sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
	}
	static float ScalarProduct(const Point &a, const Point &b)
	{
		return a.x*b.x + a.y*b.y;
	}
	float x;	// [mm]
	float y;	// [mm]
};