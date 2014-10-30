#include "Line.h"
#include "Common.h"
#include <algorithm>

using namespace std;
using namespace PathPlanner;

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

bool Line::LineSegmentIntersection(Config q1, Line s1, Point &intersect)
{
	float A = sinf(q1.phi);
	float B = cosf(q1.phi);
	float D = s1.b.y - s1.a.y;
	float E = s1.b.x - s1.a.x;

	//Intersection point of the two lines
	if ((A*E - D*B) == 0.0f)
		return false;
	else
	{
		intersect.x = (A*E*q1.p.x - B*E*q1.p.y - D*B*s1.a.x + B*E*s1.a.y) / (A*E - D*B);
		intersect.y = (B*D*q1.p.y - A*D*q1.p.x - E*A*s1.a.y + A*D*s1.a.x) / (B*D - E*A);
	}

	return s1.CheckPointInSegment(intersect);
}

bool Line::LineLineIntersection(Line l1, Line l2, Point &intersect)
{
	float A = l1.b.y - l1.a.y;
	float B = l1.b.x - l1.a.x;
	float D = l2.b.y - l2.a.y;
	float E = l2.b.x - l2.a.x;	

	//Intersection point of the two lines
	if ((A*E - D*B) == 0.0f)
		return false;
	else
	{
		intersect.x = (A*E*l1.a.x - B*E*l1.a.y - D*B*l2.a.x + B*E*l2.a.y) / (A*E - D*B);
		intersect.y = (B*D*l1.a.y - A*D*l1.a.x - E*A*l2.a.y + A*D*l2.a.x) / (B*D - E*A);
		return true;
	}
}

int Line::CircleLineIntersect(Line l1, float radius, Point center, Point &res0, Point &res1)
{
	//http://mathworld.wolfram.com/Circle-LineIntersection.html
	float dx, dy, dr, D, disc;
	int ret;

	//Transform to (0,0)
	l1.a = l1.a - center;
	l1.b = l1.b - center;

	dx = l1.b.x - l1.a.x;
	dy = l1.b.y - l1.a.y;
	dr = sqrtf(dx*dx + dy*dy);
	D = l1.a.x*l1.b.y - l1.b.x*l1.a.y;
	disc = radius*radius*dr*dr - D*D;

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
		res0.x =  D*dy + sgnC(dy)*dx*sqrtf(disc);
		res1.x =  D*dy - sgnC(dy)*dx*sqrtf(disc);
		res0.x /= (dr*dr);
		res1.x /= (dr*dr);

		res0.y = -D*dx + fabs(dy)*sqrtf(disc);
		res1.y = -D*dx - fabs(dy)*sqrtf(disc);
		res0.y /= (dr*dr);
		res1.y /= (dr*dr);

		ret = 2;
	}

	//Transform back to center
	res0 = res0 + center;
	res1 = res1 + center;
	return ret;
}
