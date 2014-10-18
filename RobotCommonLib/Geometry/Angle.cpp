#include "Angle.h"
#include "Common.h"

using namespace PathPlanner;

float Angle::Corrigate(float angle)
{
	while (angle > PI)
		angle -= 2*PI;

	while (angle < -PI)
		angle += 2*PI;

	return angle;
}

float Angle::DirectedAngleDist(float thetaStart, float thetaEnd, int turnDir)
{
	float dist = Corrigate(thetaEnd - thetaStart);
	
	if (dist == 0.0f)
	{
		return dist;
	}
	else if ( sgn(turnDir) != sgn(dist))
	{
		dist = sgn(turnDir)*(2*PI - fabs(dist)); 
	}

	return dist;
}