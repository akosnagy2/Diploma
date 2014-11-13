#include "Point.h"

using namespace PathPlanner;

Point::Point(float x, float y)
{
	this->x = x;
	this->y = y;
}
bool Point::operator==(const Point &b) const
{
	if((this->x == b.x) && (this->y == b.y))
		return true;
	else
		return false;
}
const Point Point::operator+(const Point &b) const
{
	return Point(this->x + b.x, this->y + b.y);
}
const Point Point::operator-(const Point &b) const
{
	return Point(this->x - b.x, this->y - b.y);;
}
const Point Point::operator*(const float mul) const
{
	return Point(this->x*mul, this->y*mul);
}
const Point Point::operator/(const float div) const
{
	return Point(this->x / div, this->y / div);
}
float Point::Distance(const Point &a, const Point &b)
{
	return sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}
float Point::ScalarProduct(const Point &a, const Point &b)
{
	return a.x*b.x + a.y*b.y;
}
float Point::atan2(const Point &a, const Point &b)
{
	return atan2f(a.y - b.y, a.x - b.x);
}

Json::Value Point::getJsonValue()
{
	Json::Value val;
	val["x"] = this->x;
	val["y"] = this->y;
	return val;
}

void Point::setFromJson(Json::Value& value)
{
	this->x = value["x"].asFloat();
	this->y = value["y"].asFloat();
}