#include "path_planner_funcs.h"   

void transformRobotShape(Polygon &robotShapeLocal, Polygon &robotShapeWorld, Config q)
{
	ublas::matrix<float> T = ublas::identity_matrix<float>(3);
	T(0,0) = cosf(q.phi);
	T(0,1) = -sinf(q.phi);
	T(0,2) = q.p.x;

	T(1,0) = sinf(q.phi);
	T(1,1) = cosf(q.phi);
	T(1,2) = q.p.y;
	ublas::vector<float> col(3);

	for (int i = 0; i < (int)robotShapeLocal.ps.size(); i++)
	{
		col[0] = robotShapeLocal.ps[i].x;
		col[1] = robotShapeLocal.ps[i].y;
		col[2] = 1.0;

		ublas::vector<float> prod = ublas::prod(T, col);
		robotShapeWorld.AddPoint(Point(prod[0], prod[1]));
	}
}

bool Scene::RTRPlanner()
{
	InitRTTrees();
	Point gp(26.5f, 3.0f);
	ALCCandidate alc;
	
	alc = GetALC(startTree, gp, true);

	return true;
}

ALCCandidate Scene::GetALC(Tree &tree, Point &GP, bool preferredDir)
{
	vector<ALCCandidate> ALCCandidates;
	ALCCandidates.reserve((int)tree.xtree.size() - 1);

	//Fill ALC candidate list, while taking care of smaller position and angular distance 
	float minPosDist = numeric_limits<float>::infinity(), minAngDist = numeric_limits<float>::infinity();
	vector<ALCCandidate>::iterator min;
	for (vector<TreeElement>::iterator it = tree.xtree.begin() + 1; it != tree.xtree.end(); ++it)
	{
		ALCCandidates.push_back(it->ci.PointCIDistance(GP, preferredDir));

		if (ALCCandidates.back().posDist < minPosDist) //Check posDist
		{
			minPosDist = ALCCandidates.back().posDist;
			minAngDist = fabs(ALCCandidates.back().angDist);
			min = ALCCandidates.end() - 1;
		}
		else if (ALCCandidates.back().posDist == minPosDist)
		{
			if (fabs(ALCCandidates.back().angDist) < minAngDist) //Check angDist
			{
				minPosDist = ALCCandidates.back().posDist;
				minAngDist = fabs(ALCCandidates.back().angDist);
				min = ALCCandidates.end() - 1;
			}
		}
	}

	int treeIdx = (min - ALCCandidates.begin()) + 1; //Get position in tree

	if ((min->internalC) || (min->q == tree.xtree[treeIdx].q))
		min->treeElementIdx = treeIdx;
	else
		min->treeElementIdx = tree.xtree[treeIdx].parentIdx;

	return *min;
}

void Scene::InitRTTrees()
{
	//Init Start, Goal trees
	startTree.AddElement(TreeElement(robotStart), -1);
	goalTree.AddElement(TreeElement(robotGoal), -1);

	//TCI extension of the start config
	ConfigInterval startFwdCI, startBwdCI;	
	TCI_Extension(robotStart, startFwdCI, startBwdCI); 

	startTree.AddElement(TreeElement(startFwdCI), 0);
	startTree.AddElement(TreeElement(startBwdCI), 0);

	//TCI extension of the goal config
	ConfigInterval goalFwdCI, goalBwdCI;	
	TCI_Extension(robotGoal, goalFwdCI, goalBwdCI); 

	goalTree.AddElement(TreeElement(goalFwdCI), 0);
	goalTree.AddElement(TreeElement(goalBwdCI), 0);
}

void Scene::TCI_Extension(Config q, ConfigInterval &forwardMaxTCI, ConfigInterval &backwardMaxTCI)
{
	//Transform the robot polygon to the world frame
	Polygon robotShapeWorld;
	transformRobotShape(robotShape, robotShapeWorld, q);
	
	//Traverse on obstacles
	vector<float> forwardPointDist, backwardPointDist;
	for (int i = 0; i < (int)envsx.size(); i++)
	{
		int env_size = envsx[i].ps.size();
		for (int j = 0; j < env_size; j++) //Traverse on obstacles's edges
		{
			int rob_size = robotShapeWorld.ps.size();
			for (int k = 0; k < rob_size; k++) //Traverse on robot's edges
			{
				//Collision check of robot polygon points with obstacle edges				
				Point p0(envsx[i].ps[j].x, envsx[i].ps[j].y);
				Point p1(envsx[i].ps[((j + 1) % env_size)].x, envsx[i].ps[((j + 1) % env_size)].y);
				Config q0(robotShapeWorld.ps[k],q.phi);
				Point inter;

				if (lineSegmentIntersection(q0, p0, p1, inter))
				{
					float alfa = atan2f(inter.y - q0.p.y, inter.x - q0.p.x);

					if (fabs(corrigateAngle(alfa - q0.phi)) < EPS) //Forward collision possibility
					{
						forwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
					else if (fabs(corrigateAngle(-(PI - alfa) - q0.phi)) < EPS) //Backward collision possibility
					{
						backwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
				}

				//Collision check of obstacle corner points with robot polygon edges
				p0.x = robotShapeWorld.ps[k].x;
				p0.y = robotShapeWorld.ps[k].y;
				p1.x = robotShapeWorld.ps[((k + 1) % rob_size)].x;
				p1.y = robotShapeWorld.ps[((k + 1) % rob_size)].y;
				q0.p = envsx[i].ps[j];				 

				if (lineSegmentIntersection(q0, p0, p1, inter))
				{
					float alfa = atan2f(q0.p.y - inter.y, q0.p.x - inter.x);

					if (fabs(corrigateAngle(alfa - q0.phi)) < EPS) //Forward collision possibility
					{
						forwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
					else if (fabs(corrigateAngle(-(PI - alfa) - q0.phi)) < EPS) //Backward collision possibility
					{
						backwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
				}
			}
		}
	}

	//Determine the nearest intersection point in forward and backward directions
	float forwardMinDist = *min_element(forwardPointDist.begin(), forwardPointDist.end());
	float backwardMinDist = *min_element(backwardPointDist.begin(), backwardPointDist.end());

	forwardMinDist = forwardMinDist - min(0.1f*forwardMinDist, EPS);
	backwardMinDist = backwardMinDist - min(0.1f*backwardMinDist, EPS);
	if (forwardMinDist < EPS)
		forwardMinDist = 0.0f;
	if (backwardMinDist < EPS)
		backwardMinDist = 0.0f;

	//Set TCIs
	forwardMaxTCI.type = TranslationCI;
	forwardMaxTCI.q0 = q;
	forwardMaxTCI.q1 = q + Config(forwardMinDist*cosf(q.phi), forwardMinDist*sinf(q.phi), 0.0f);
	forwardMaxTCI.amount = forwardMinDist;

	backwardMaxTCI.type = TranslationCI;
	backwardMaxTCI.q0 = q;
	backwardMaxTCI.q1 = q - Config(backwardMinDist*cosf(q.phi), backwardMinDist*sinf(q.phi), 0.0f);
	backwardMaxTCI.amount = -backwardMinDist;
}