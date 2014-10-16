#include "Tree.h"
#include "..\Geometry\Angle.h"

using namespace PathPlanner;

Tree::Tree()
{}

int Tree::AddElement(TreeElement &elem, int parentIdx)
{
	if (parentIdx >= (int)xtree.size())
		return -1;

	elem.parentIdx = parentIdx;
	xtree.push_back(elem);

	int id = (int)(xtree.size()) - 1;
	if (parentIdx >= 0)
		xtree[parentIdx].childrenIdx.push_back(id);		
		
	return id;
}

int Tree::Split(int splitID, Config &q)
{
	TreeElement elem;

	//Inserting a new tree element at splitID
	elem.childrenIdx.push_back(splitID);
	elem.parentIdx = xtree[splitID].parentIdx;
	elem.q = q;
	elem.ci.type = xtree[splitID].ci.type;
	elem.ci.q0 = xtree[splitID].ci.q0;
	elem.ci.q1 = q;
	if (elem.ci.type == TranslationCI)
		elem.ci.amount = (sgn(xtree[splitID].ci.amount)) * Config::Distance(elem.ci.q0, elem.ci.q1);
	else
		elem.ci.amount = Angle::DirectedAngleDist(elem.ci.q0.phi, elem.ci.q1.phi, sgn(xtree[splitID].ci.amount));

	xtree.push_back(elem);
	int id = (int)xtree.size() - 1;

	//Modifying the children list of the parent of the old tree element whose CI was splitted
	(*find(xtree[elem.parentIdx].childrenIdx.begin(), xtree[elem.parentIdx].childrenIdx.end(), splitID)) = id;

	//Modifying the old tree element whose CI was splitted
	xtree[splitID].parentIdx = id;
	xtree[splitID].ci.q0 = q;
	xtree[splitID].ci.amount -= elem.ci.amount; 

	return id;
}

vector<ConfigInterval> Tree::PathFromRoot(int id)
{
	vector<ConfigInterval> path;

	if (id > 0)
	{
		//Fill elements of the path from the given element to the root
		do
		{
			path.push_back(xtree[id].ci);
			id = xtree[id].parentIdx;
		} while (xtree[id].parentIdx != -1);

		reverse(path.begin(), path.end());
	}
	return path;
}