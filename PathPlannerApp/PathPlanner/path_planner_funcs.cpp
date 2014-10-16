/*
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

//TODO: lehetne távolság alapján is, csak gyök nélkül
inline bool checkPointInSegment(Point p, Point s1, Point s2)
{
	if ( (min(s1.x, s2.x) <= p.x) && (min(s1.y, s2.y) <= p.y) && (p.x <= max(s1.x, s2.x)) && (p.y <= max(s1.y, s2.y)) )
		return true;
	else
		return false;
}

//Ez is pont-tal menjen, számolja ki a hívó fél
bool lineSegmentIntersection(Config line, Point s1, Point s2, Point &intersect)
{
	if (lineLineIntersection(Point(0.0f, 0.0f), Point(cosf(line.phi), sinf(line.phi)), s1, s2, intersect))
		return checkPointInSegment(intersect, s1, s2);
}

bool segmentSegmentItersection(Point s1, Point s2, Point s3, Point s4, Point &intersect)
{
	if (lineLineIntersection(s1, s2, s3, s4, intersect))
		return (checkPointInSegment(intersect, s1, s2) && checkPointInSegment(intersect, s3, s4));
}

bool lineLineIntersection(Point s1, Point s2, Point s3, Point s4, Point &intersect)
{
	float A = s2.y - s1.y;
	float B = s2.x - s1.x;
	float D = s4.y - s3.y;
	float E = s4.x - s3.x;
	
	float det = A*E - D*B;

	//Intersection point of the two lines
	if (det == 0.0f)
		return false;
	else
	{
		intersect.x = (E*(A*s1.x - B*s1.y) - B*(D*s3.x - E*s3.y)) / det;
		intersect.y = (A*(E*s3.y - D*s3.x) - D*(B*s1.y - A*s1.x)) / det;

		return true;
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

int circleLineIntersect(Point p1, Point p2, float radius, Point center, Point &res0, Point &res1)
{
	float dx, dy, dr, D, disc;
	int ret;
	p1 = p1 - center;
	p2 = p2 - center;

	dx = p2.x - p1.x;
	dy = p2.y - p1.y;
	dr = sqrtf(dx*dx + dy*dy);
	D = p1.x*p2.y - p2.x*p1.y;
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
		res0.x = sgn(dy)*dx*sqrtf(disc) + D*dy;
		res1.x = -res0.x + D*dy;
		res0.x /= powf(dr,2);
		res1.x /= powf(dr,2);

		res0.y = fabs(dy)*sqrtf(disc) - D*dx;
		res1.y = -res0.y - D*dx;
		res0.y /= powf(dr,2);
		res1.y /= powf(dr,2);

		ret = 2;
	}

	res0 = res0 + center;
	res1 = res1 + center;
	return ret;
}

/*
* Intersection point of a circle segment and a line segment
*
*   INPUTS:
*
*   qStart	-   Starting configuration [x, y, theta]
*   dtheta	-   Change in the orientation during traversing the circular
*               segment (signed, positive means CCW motion)
*   radius	-   Radius of the circular segment (signed, positive means that
*               the centerpoint of the circle is on the left side if
*               looking in direction 'theta' from (x,y)
*   s1		-   First point of the linesegment
*   s2		-   Second point of the linesegment
*
*   OUTPUT:
*
*   res		-	Intersection points
*	return	-	Number of intersections
*/
/*
int circleSegLineSegIntersect(Config qStart, float dTheta, float radius, Point s1, Point s2, Point res[2])
{
	Point center(qStart.p.x - radius*sinf(qStart.phi), qStart.p.y + radius*cosf(qStart.phi));

	if (!circleLineIntersect(s1, s2, fabs(radius), center, res[0], res[1]))
		return 0; 

	int interNum = 0;
	float startAngle = corrigateAngle(qStart.phi - sgn(radius)*PI*0.5f);

	for (int i = 0; i < 2; i++)
	{
		float checkAngle = Point::atan2(res[i], center) - startAngle;

		if (sgn(checkAngle) != sgn(dTheta))
			checkAngle = sgn(dTheta)*(2*PI + sgn(dTheta)*checkAngle);

		if ((fabs(dTheta) >= fabs(checkAngle)) && checkPointInSegment(res[i], s1, s2))
			interNum++;
	}

	return interNum;
}
*/