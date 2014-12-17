#include "Config.h"

using namespace PathPlanner;

/*PointAngularDistance:
* Signed smallest angular distance of a point and a configuration 
*
* The angular distance is measured assuming a preferred reference orientation, given by the input parameter 
*   fromConfToPoint  -   positive: reference orientation points from the configuration position to the given point
*						 negative: reference orientation points in the opposite direction
*/
float Config::PointAngularDistance(const Point &p, bool fromConfToPoint)
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

Json::Value Config::getJsonValue()
{
	Json::Value val;
	val["x"] = this->p.x;
	val["y"] = this->p.y;
	val["phi"] = this->phi;
	return val;
}

void Config::setFromJson(Json::Value& value)
{
	this->p.x = value["x"].asFloat();
	this->p.y = value["y"].asFloat();
	this->phi = value["phi"].asFloat();
}