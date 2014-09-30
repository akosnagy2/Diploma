#pragma once
#include "Config.h"

using namespace std;

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

	/* Position and angular distance calculation between a position and a CI,
	*  returning the configuration with the smallest distance
	*	TCI: the configuration with the smallest position distance is returned
	*   RCI: the configuration with the smallest (signed) angular distance is returned
	*  It is also returned whether the nearest configuration is an internal
	*  configuration of the CI (return true), or one of its end configurations (return false)
	*
	*  The angular distance is measured assuming a preferred reference orientation, given by the input parameter 
	*	preferredOrient  -  positive: reference orientation points to the GP
	*                       negative: reference orientation points away from the GP
	*/
	ALCCandidate PointCIDistance(Point &p, bool preferredOrient)
	{
		if (type == TranslationCI)
			return PointTCIDistance(p, preferredOrient);
		else
			return PointRCIDistance(p, preferredOrient);
	}

	//Position and angular distance calculation between a position and a TCI, returning the configuration with the smallest position distance
	ALCCandidate PointTCIDistance(const Point &p, bool preferredOrient)
	{
		ALCCandidate alc;
		Point proj;

		//Determine the position distance and the nearest configuration
		if (pointProjectToSegment(p, q0.p, q1.p, proj))
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

	//Position and angular distance calculation between a position and an RCI, returning the configuration with the smallest (signed) angular distance 
	ALCCandidate PointRCIDistance(const Point &p, bool preferredOrient)
	{
		ALCCandidate alc;
		float refAngle;

		//Determine the reference angle (pointing to or away from the given point, accorting to preferred_ori)
		if (preferredOrient)
			refAngle = atan2f(p.y - q0.p.y, p.x - q0.p.x);
		else
			refAngle = atan2f(p.y - q0.p.y, p.x - q0.p.x);

		//Determine whether the reference angle is an internal angle of the RCI
		alc.internalC =  (fabs(directedAngleDist(q0.phi, refAngle, (amount > 0.0f))) < fabs(amount));
			
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
};

struct TreeElement 
{
	//Ha kell törölni, akkor inkább külön jelezzük, hogy érvényes-e!!!!!!!!!!!!!!!!!!!
	//int					ID;			//Identifier
	Config					q;				//Corresponding configuration
	int						parentIdx;		//Parent tree element index in the tree
	vector<int>				childrenIdx;	//Children tree element index in the tree
	ConfigInterval			ci;				//Configuration interval leading from parent to here
	TreeElement()
	{}
	TreeElement(Config q)
	{
		this->q = q;
		this->parentIdx = -1;
	}
	TreeElement(Config q, ConfigInterval &ci) //Create Tree Element with Config, ConfigInterval
	{
		this->q = q;
		this->ci = ci;
		this->parentIdx = -1;
	}
	TreeElement(ConfigInterval &ci) //Create Tree Element with ConfigInterval
	{
		this->q = ci.q1;
		this->ci = ci;
		this->parentIdx = -1;
	}
};

struct Tree
{
	vector<TreeElement> xtree;
	Tree()
	{}
	bool AddElement(TreeElement &elem, int parentIdx)
	{
		if (parentIdx >= (int)xtree.size())
			return false;

		elem.parentIdx = parentIdx;
		xtree.push_back(elem);

		if (parentIdx >= 0)
			xtree[parentIdx].childrenIdx.push_back((int)(xtree.size()) - 1);		
		
		return true;
	}
};