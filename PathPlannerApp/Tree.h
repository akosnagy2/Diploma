#pragma once
#include "Config.h"

using namespace std;

enum IntervalType
{
	TranslationCI,
	RotationCI
};

struct ConfigInterval
{
	IntervalType		type;	//Configuration interval type
	Config				q0;		//Initial configuration of the CI
	Config				q1;		//Final configuration of the CI
	float				amount; //Signed movement amount among the CI (displacement or turning angle)

	bool PointCIDistance(const Point &p, bool preferedOrient, Config &nearestC, float &posDist, float &angDist)
	{
		if (type == TranslationCI)
			return PointTCIDistance(p, preferedOrient, nearestC, posDist, angDist);
		else
			return PointRCIDistance(p, preferedOrient, nearestC, posDist, angDist);
	}
	bool PointTCIDistance(const Point &p, bool preferedOrient, Config &nearestC, float &posDist, float &angDist)
	{
	}
	bool PointRCIDistance(const Point &p, bool preferedOrient, Config &nearestC, float &posDist, float &angDist)
	{
	}
};

struct TreeElement 
{
	//Ha kell törölni, akkor inkább külön jelezzük, hogy érvényes-e!!!!!!!!!!!!!!!!!!!
	//int					ID;			//Identifier
	Config					q;			//Corresponding configuration
	int						parentIdx;		//Parent tree element pointer
	vector<int>				childrenIdx;	//Children tree element pointers
	ConfigInterval			ci;			//Configuration interval leading from parent to here
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
