#include "Line.h"
#include "path_planner_funcs.h"
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

	if (fabs(A*E - D*B) > 0.0f)
	{
		intersect.x = (A*E*q1.p.x - B*E*q1.p.y - D*B*s1.a.x + B*E*s1.a.y) / (A*E - D*B);
		intersect.y = (B*D*q1.p.y - A*D*q1.p.x - E*A*s1.a.y + A*D*s1.a.x) / (B*D - E*A);

		if ((((fabs(intersect.x - s1.a.x) + fabs(intersect.x - s1.b.x))) - fabs(E)) < 0.001)
		{
			if ((((fabs(intersect.y - s1.a.y) + fabs(intersect.y - s1.b.y))) - fabs(D)) < 0.001)
			{
				if ( (fabs(intersect.x - s1.a.x) > 0.001) || (fabs(intersect.y - s1.a.y) > 0.001) )
				{
					if ( (fabs(intersect.x - s1.b.x) > 0.001) || (fabs(intersect.y - s1.b.y) > 0.001) )
						return true;
				}
			}
		}
	}

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

int Line::CircleLineIntersect(Config q1, float radius, Point center, Point &res0, Point &res1)
{
	float v1 = cosf(q1.phi);
	float v2 = sinf(q1.phi);

	float k = v2*q1.p.x - v1*q1.p.y;

	if (fabs(v2) > 0.0001) ///!!!!!!!!!!!!
	{
		float A = v1/v2 * v1/v2 + 1;
		float B = 2 * (k/v2 - center.x) * (v1/v2) - 2*center.y;
		float C = (k/v2 - center.x)*(k/v2 - center.x) + center.y*center.y - radius*radius;

		if ((B*B - 4*A*C) > 0.0)
		{
			res0.y = (-B + sqrtf(B*B - 4*A*C)) / (2*A);
			res1.y = (-B - sqrtf(B*B - 4*A*C)) / (2*A);

			res0.x = (k + v1*res0.y) / v2;
			res1.x = (k + v1*res1.y) / v2;

			return 2;
		}
	}
	else
	{
		float A = v2/v1 * v2/v1 + 1;
		float B = -2 * (k/v1 + center.y) * (v2/v1) - 2*center.x;
		float C = (k/v1 + center.y)*(k/v1 + center.y) + center.x*center.x - radius*radius;

		if ((B*B - 4*A*C) > 0.0)
		{
			res0.x = (-B + sqrtf(B*B - 4*A*C)) / (2*A);
			res1.x = (-B - sqrtf(B*B - 4*A*C)) / (2*A);

			res0.y = (k - v2*res0.y) / (-v1);
			res1.y = (k - v2*res1.y) / (-v1);

			return 2;
		}
	}

	return 0;
}