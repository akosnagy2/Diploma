#pragma once
#include <vector>
#include "Point.h"
#include "Config.h"
#include "ISerializable.h"

using namespace std;

namespace PathPlanner
{

	struct Polygon : public ISerializable
	{
		Polygon() {}
		Polygon(float width, float heigth);
		void AddPoint(Point p)
		{
			ps.push_back(p);
		}
		Polygon TransformToWorld(Config q);
		Polygon TransformToLocal(Config q);

		float getWidth();
		float getHeight();
		vector<Point>& getPoints() { return ps; }

		vector<Point> ps;

		virtual Json::Value getJsonValue();
		virtual void setFromJson(Json::Value& value);
	};

}