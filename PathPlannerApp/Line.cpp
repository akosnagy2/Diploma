#include "Line.h"
#include "path_planner_funcs.h"
#include <algorithm>

using namespace std;

float Line::Length()
{
	return Point::Distance(a, b);
}

bool Line::CheckPointInSegment(Point p)
{
	if ( (min(a.x, b.x) - EPS <= p.x) && (min(a.y, b.y) - EPS <= p.y) && (p.x <= max(a.x, b.x) + EPS) && (p.y <= max(a.y, b.y) + EPS) )
		return true;
	else
		return false;
}

bool Line::ProjectPointToSegment(Point p, Line s1, Point &proj)
{
	float segmentLength = s1.Length();
	float scalarTmp = Point::ScalarProduct(p - s1.a, s1.b - s1.a) / segmentLength;

	proj = s1.a + (s1.b - s1.a) * scalarTmp / segmentLength;

	return s1.CheckPointInSegment(proj);
}

bool Line::SegmentSegmentItersection(Line s1, Line s2, Point &intersect)
{
	if (LineLineIntersection(s1, s2, intersect))
		return (s1.CheckPointInSegment(intersect) && s2.CheckPointInSegment(intersect));
	else
		return false;
}

bool Line::LineSegmentIntersection(Line l1, Line s1, Point &intersect)
{
	if (LineLineIntersection(l1, s1, intersect))
		return s1.CheckPointInSegment(intersect);
	else
		return false;
}

bool Line::LineLineIntersection(Line l1, Line l2, Point &intersect)
{
	float A = l1.b.y - l1.a.y;
	float B = l1.b.x - l1.a.x;
	float D = l2.b.y - l2.a.y;
	float E = l2.b.x - l2.a.x;
	
	float det = A*E - D*B;

	//Intersection point of the two lines
	if (det == 0.0f)
		return false;
	else
	{
		intersect.x = (E*(A*l1.a.x - B*l1.a.y) - B*(D*l2.a.x - E*l2.a.y)) / det;
		intersect.y = (A*(E*l2.a.y - D*l2.a.x) - D*(B*l1.a.y - A*l1.a.x)) / det;

		return true;
	}
}

int Line::CircleLineIntersect(Line l1, float radius, Point center, Point &res0, Point &res1)
{
	float dx, dy, dr, D, disc;
	int ret;
	l1.a = l1.a - center;
	l1.b = l1.b - center;

	dx = l1.b.x - l1.a.x;
	dy = l1.b.y - l1.a.y;
	dr = sqrtf(dx*dx + dy*dy);
	D = l1.a.x*l1.b.y - l1.b.x*l1.a.y;
	disc = powf(radius,2)*powf(dr,2) - powf(D,2);

	//No intersection
	if (disc < 0.0f)
		return 0;
	else if (disc == 0.0f)	//Tangent (one intersection)
	{
		res0.x = D*dy/powf(dr,2);
		res0.y = -D*dx/powf(dr,2);
		res1 = res0;
		ret = 1;
	}
	else   //Two intersection
	{
		res0.x = (int)(dy >= 0)*dx*sqrtf(disc) + D*dy;
		res1.x = -(int)(dy >= 0)*dx*sqrtf(disc) + D*dy;
		res0.x /= powf(dr,2);
		res1.x /= powf(dr,2);

		res0.y = fabs(dy)*sqrtf(disc) - D*dx;
		res1.y =  -fabs(dy)*sqrtf(disc) - D*dx;
		res0.y /= powf(dr,2);
		res1.y /= powf(dr,2);

		ret = 2;
	}

	res0 = res0 + center;
	res1 = res1 + center;
	return ret;
}