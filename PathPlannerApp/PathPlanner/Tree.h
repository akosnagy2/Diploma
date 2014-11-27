#pragma once

#include "Geometry\Config.h"
#include "ConfigInterval.h"
#include "Geometry\Line.h"
#include "Geometry\Common.h"
#include <vector>

using namespace std;

namespace PathPlanner
{
	struct TreeElement 
	{
		//Ha kell törölni, akkor inkább külön jelezzük, hogy érvényes-e!!!!!!!!!!!!!!!!!!!
		//int					ID;			//Identifier
		//Config					q;				//Corresponding configuration
		int						parentIdx;		//Parent tree element index in the tree
		vector<int>				childrenIdx;	//Children tree element index in the tree
		ConfigInterval			ci;				//Configuration interval leading from parent to here
		TreeElement()
		{}
		TreeElement(Config q)
		{
			this->ci.amount = 0.0f;
			this->ci.type = TranslationCI;
			this->ci.q0 = q;
			this->ci.q1 = q;
			this->parentIdx = -1;
		}/*
		TreeElement(Config q, ConfigInterval &ci) //Create Tree Element with Config, ConfigInterval
		{
			//this->q = q;
			this->ci = ci;
			this->parentIdx = -1;
		}*/
		TreeElement(ConfigInterval &ci) //Create Tree Element with ConfigInterval
		{
			//this->q = ci.q1;
			this->ci = ci;
			this->parentIdx = -1;
		}
	};

	struct Tree
	{
		Tree();
		int AddElement(TreeElement &elem, int parentIdx);
		/*
		* Splitting a given tree element of a tree at the given
		* configuration
		*
		* Inputs:
		*   splitID     - Identifier of the tree element whose CI is splitted
		*   q			- Internal configuration of the CI which has to be splitted
		*
		* Retrun:
		*   newID       - The ID of the new tree element
		*/
		int Split(int splitID, Config &q);
		vector<ConfigInterval> PathFromRoot(int id);

		vector<TreeElement> xtree;
	};

}