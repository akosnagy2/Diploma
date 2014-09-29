#include "path_planner_funcs.h"   
#define _USE_MATH_DEFINES
#include <math.h>

template<typename T>
void printMat(ublas::matrix<T> mat)
{
	for (int i = 0; i < (int)mat.size1(); i++)
	{
		cout << i << ". sor:" << endl;
		for (int j = 0; j < (int)mat.size2(); j++)
		{		
			cout << mat(i, j) << " ";			
		}
		cout << endl;
	}
}

float corrigateAngle(float angle, uint16_t zero_to_2pi)
{
	float minLim, maxLim;
	float ret = angle;

	if (zero_to_2pi)
	{
		minLim = 0.0f;
		maxLim = (float)(2*M_PI);
	}
	else
	{
		minLim = -(float)M_PI;
		maxLim = (float)M_PI;
	}

	while (ret > maxLim)
		ret -= (float)(2*M_PI);

	while (ret < minLim)
		ret += (float)(2*M_PI);

	return ret;
}

bool insideTriangle(Point a, Triangle t)
{
	//http://www.blackpawn.com/texts/pointinpoly/
	
	// Compute vectors        
	Point v0 = t.p[2] - t.p[0];
	Point v1 = t.p[1] - t.p[0];
	Point v2 = a - t.p[0];

	// Compute dot products
	float dot00 = Point::Dot(v0, v0);
	float dot01 = Point::Dot(v0, v1);
	float dot02 = Point::Dot(v0, v2);
	float dot11 = Point::Dot(v1, v1);
	float dot12 = Point::Dot(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}

Zone2* generateObjectZone(Polygon &obj, Fade_2D &dt)
{
	vector<Segment2> seg;

	seg.reserve(obj.ps.size());

	//Create FADE2D segments(edges)
	for (int i = 0; i < (int)(obj.ps.size() - 1); ++i)
	{
		Point2 p0(obj.ps[i].x, obj.ps[i].y);
		Point2 p1(obj.ps[i+1].x, obj.ps[i+1].y);
		seg.push_back(Segment2(p0, p1));
	}
	Point2 p0(obj.ps.back().x, obj.ps.back().y);
	Point2 p1(obj.ps.front().x, obj.ps.front().y);
	seg.push_back(Segment2(p0, p1));

	ConstraintGraph2* cstrs = dt.createConstraint(seg, CIS_IGNORE_DELAUNAY);
	Zone2* zone = dt.createZone(cstrs, ZL_OUTSIDE);

	return zone;
}