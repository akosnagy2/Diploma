#include "Tree.h"
#include "Geometry\Angle.h"

using namespace PathPlanner;

Tree::Tree()
{}

TreeElement* Tree::AddElement(TreeElement &elem, TreeElement* parentIdx)
{
	//TreeElement t = elem;
	//if (parentIdx >= (int)xtree.size())
	//	return -1;

	if ((elem.ci.amount == 0.0) && (parentIdx != NULL))
		return NULL;

	elem.parentIdx = parentIdx;
	xtree.push_back(elem);

	//int id = (int)(xtree.size()) - 1;
	if (parentIdx != NULL)
		parentIdx->childrenIdx.push_back(&xtree.back());		
		
	return &xtree.back();
}

TreeElement*  Tree::Split(TreeElement*  splitID, Config &q)
{
	TreeElement elem;

	//Inserting a new tree element at splitID
	elem.childrenIdx.push_back(splitID);
	elem.parentIdx = splitID->parentIdx;
	//elem.q = q;
	elem.ci.type = splitID->ci.type;
	elem.ci.q0 = splitID->ci.q0;
	elem.ci.q1 = q;
	if (elem.ci.type == TranslationCI)
		elem.ci.amount = (sgn(splitID->ci.amount)) * Config::Distance(elem.ci.q0, elem.ci.q1);
	else
		elem.ci.amount = Angle::DirectedAngleDist(elem.ci.q0.phi, elem.ci.q1.phi, sgn(splitID->ci.amount));

	xtree.push_back(elem);
	TreeElement*  id = &(xtree.back());

	//Modifying the children list of the parent of the old tree element whose CI was splitted
	(*find(elem.parentIdx->childrenIdx.begin(), elem.parentIdx->childrenIdx.end(), splitID)) = id;

	//Modifying the old tree element whose CI was splitted
	splitID->parentIdx = id;
	splitID->ci.q0 = q;
	splitID->ci.amount -= elem.ci.amount; 

	return id;
}

vector<ConfigInterval> Tree::PathFromRoot(TreeElement* id)
{
	vector<ConfigInterval> path;
	TreeElement* e = id;

	if (e != NULL)
	{
		//Fill elements of the path from the given element to the root
		do
		{
			path.push_back(e->ci);
			e = e->parentIdx;
		} while (e->parentIdx != NULL);

		reverse(path.begin(), path.end());
	}
	return path;
}