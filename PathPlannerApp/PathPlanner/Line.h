#pragma once

#include "Point.h"

namespace PathPlanner
{

	struct Line
	{
		Line(Point _a, Point _b) : a(_a), b(_b) {}
		float Length();
		bool CheckPointInSegment(Point p);
		static bool ProjectPointToSegment(Point p, Line s1, Point &proj);
		static bool SegmentSegmentItersection(Line s1, Line s2, Point &intersect);
		static bool LineSegmentIntersection(Line l1, Line s1, Point &intersect);
		static bool LineLineIntersection(Line l1, Line l2, Point &intersect);
		static int CircleLineIntersect(Line l1, float radius, Point center, Point &res0, Point &res1);

		Point a;
		Point b;
	};

}