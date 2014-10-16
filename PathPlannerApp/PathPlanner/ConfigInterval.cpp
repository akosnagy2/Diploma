#include "ConfigInterval.h"
#include "Geometry\Common.h"

using namespace PathPlanner;
using namespace std;

ALCCandidate ConfigInterval::PointCIDistance(Point &p, bool preferredOrient)
{
	if (type == TranslationCI)
		return PointTCIDistance(p, preferredOrient);
	else
		return PointRCIDistance(p, preferredOrient);
}

ALCCandidate ConfigInterval::PointTCIDistance(const Point &p, bool preferredOrient)
{
	ALCCandidate alc;
	Point proj;

	//Determine the position distance and the nearest configuration
	if (Line::ProjectPointToSegment(p, Line(q0.p, q1.p), proj))
	{
		//Internal config
		alc.internalC = true;

		alc.posDist = Point::Distance(p, proj);
		alc.q.p = proj;
		alc.q.phi = q0.phi;
	}
	else
	{
		//Boundary config
		alc.internalC = false;

		float dist0 = Point::Distance(p, q0.p);
		float dist1 = Point::Distance(p, q1.p);

		if (dist0 < dist1)
		{
			alc.posDist = dist0;
			alc.q = q0;
		}
		else
		{
			alc.posDist = dist1;
			alc.q = q1;			
		}
	}

	//Determine angular distance (trivial)
	alc.angDist = alc.q.PointAngularDistance(p, preferredOrient);
	return alc;
}

ALCCandidate ConfigInterval::PointRCIDistance(const Point &p, bool preferredOrient)
{
	ALCCandidate alc;
	float refAngle;

	//Determine the reference angle (pointing to or away from the given point, accorting to preferred_ori)
	if (preferredOrient)
		refAngle = Point::atan2(p, q0.p);
	else
		refAngle = Point::atan2(q0.p, p);

	//Determine whether the reference angle is an internal angle of the RCI
	alc.internalC =  (fabs(Angle::DirectedAngleDist(q0.phi, refAngle, sgn(amount))) < fabs(amount));
			
	//Determine the angular distance and the nearest configuration
	if (alc.internalC)
	{
		alc.angDist = 0.0f;

		alc.q.p = q0.p;
		alc.q.phi = refAngle;
	}
	else
	{
		float angDist0 = q0.PointAngularDistance(p, preferredOrient);
		float angDist1 = q1.PointAngularDistance(p, preferredOrient);

		if (fabs(angDist0) < fabs(angDist1))
		{
			alc.angDist = angDist0;
			alc.q = q0;
		}
		else
		{
			alc.angDist = angDist1;
			alc.q = q1;
		}
	}

	//Determine position distance (trivial)
	alc.posDist = Point::Distance(p, q0.p);
	return alc;
}