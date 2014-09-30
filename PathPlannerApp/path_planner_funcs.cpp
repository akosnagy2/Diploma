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

float corrigateAngle(float angle)
{
	while (angle > PI)
		angle -= 2*PI;

	while (angle < -PI)
		angle += 2*PI;

	return angle;
}

bool insideTriangle(Point a, Triangle t)
{
	//http://www.blackpawn.com/texts/pointinpoly/
	
	// Compute vectors        
	Point v0 = t.p[2] - t.p[0];
	Point v1 = t.p[1] - t.p[0];
	Point v2 = a - t.p[0];

	// Compute dot products
	float dot00 = Point::ScalarProduct(v0, v0);
	float dot01 = Point::ScalarProduct(v0, v1);
	float dot02 = Point::ScalarProduct(v0, v2);
	float dot11 = Point::ScalarProduct(v1, v1);
	float dot12 = Point::ScalarProduct(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}

inline bool checkPointInSegment(Point p, Point s1, Point s2)
{
	if ( (min(s1.x, s2.x) <= p.x) && (min(s1.y, s2.y) <= p.y) && (p.x <= max(s1.x, s2.x)) && (p.y <= max(s1.y, s2.y)) )
		return true;
	else
		return false;
}

bool lineSegmentIntersection(Config line, Point s1, Point s2, Point &intersect)
{
	float A = sinf(line.phi);
	float B = cosf(line.phi);
	float D = s2.y - s1.y;
	float E = s2.x - s1.x;

	float det = A*E - D*B;

	//Intersection point of the two lines
	if (det == 0.0f)
		return false;
	else
	{
		intersect.x = (E*(A*line.p.x - B*line.p.y) - B*(D*s1.x - E*s1.y)) / det;
		intersect.y = (A*(E*s1.y - D*s1.x) - D*(B*line.p.y - A*line.p.x)) / det;

		return checkPointInSegment(intersect, s1, s2);
	}
}



bool pointProjectToSegment(Point p, Point s1, Point s2, Point &proj)
{
	float segmentLength = Point::Distance(s1, s2);
	float scalarTmp = Point::ScalarProduct(p - s1, s2 - s1) / segmentLength;

	proj = s1 + (s2 - s1) * scalarTmp / segmentLength;

	return checkPointInSegment(proj, s1, s2);
}

float directedAngleDist(float thetaStart, float thetaEnd, bool turnDir)
{
	float dist = corrigateAngle(thetaEnd - thetaStart);
	
	if (dist == 0.0f)
	{
		return dist;
	}
	else if ( turnDir != (dist > 0.0f))
	{
		if (turnDir)
			dist = 2*PI - fabs(dist); 
		else
			dist = -(2*PI - fabs(dist)); 
	}

	return dist;
}