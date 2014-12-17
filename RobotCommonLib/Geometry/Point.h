#pragma once

#include <math.h>
#include "ISerializable.h"

namespace PathPlanner
{

	struct Point : ISerializable
	{
		Point() {}
		Point(float x, float y);

		bool operator==(const Point &b) const;
		const Point operator+(const Point &b) const;
		const Point operator-(const Point &b) const;
		const Point operator*(const float mul) const;
		const Point operator/(const float div) const;
		static float Distance(const Point &a, const Point &b);
		static float Distance2(const Point &a, const Point &b);
		static float ScalarProduct(const Point &a, const Point &b);
		static float atan2(const Point &a, const Point &b);

		float x;	// [mm]
		float y;	// [mm]

		virtual Json::Value getJsonValue();
		virtual void setFromJson(Json::Value& value);
	};

}