#pragma once

#include "Config.h"
#include "Point.h"
#include "Line.h"

enum IntervalType
{
	TranslationCI,
	RotationCI
};

struct ALCCandidate
{
	Config	q;
	float	posDist;
	float	angDist;
	bool	internalC;
	int		treeElementIdx;
};


struct ConfigInterval
{
	IntervalType		type;	//Configuration interval type
	Config				q0;		//Initial configuration of the CI
	Config				q1;		//Final configuration of the CI
	float				amount; //Signed movement amount among the CI (displacement or turning angle)

	/*
	* Position and angular distance calculation between a position and a CI,
	* returning the configuration with the smallest distance
	*	TCI: the configuration with the smallest position distance is returned
	*   RCI: the configuration with the smallest (signed) angular distance is returned
	* It is also returned whether the nearest configuration is an internal
	* configuration of the CI (return true), or one of its end configurations (return false)
	*
	* The angular distance is measured assuming a preferred reference orientation, given by the input parameter 
	*	preferredOrient  -  positive: reference orientation points to the GP
	*                       negative: reference orientation points away from the GP
	*/
	ALCCandidate PointCIDistance(Point &p, bool preferredOrient);

	//Position and angular distance calculation between a position and a TCI, returning the configuration with the smallest position distance
	ALCCandidate PointTCIDistance(const Point &p, bool preferredOrient);

	//Position and angular distance calculation between a position and an RCI, returning the configuration with the smallest (signed) angular distance 
	ALCCandidate PointRCIDistance(const Point &p, bool preferredOrient);
};
