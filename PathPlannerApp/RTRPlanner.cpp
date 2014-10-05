#include "Scene.h"
#include "Config.h"
#include "Point.h"
#include "Line.h"
#include "path_planner_funcs.h"
#include <queue>
#include <random>

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
	
	for (int i = 0; i < maxIteration; i++)
	{
		RTRIteration(true);
		RTRIteration(false);

		if (MergeTreesGetPath())
			break;
	}

	return true;
}

void Scene::RTRIteration(bool start)
{
	//Select tree
	Tree &tree = (start) ? startTree : goalTree;
	vector<int> &recentTCIIDs = (start) ? recentTCIStartIDs : recentTCIGoalIDs;

	//Choose a guiding position (GP)
	Point gp = GetGuidePoint(start);

	//Choose an active local lonfiguration (ALC) in the actual RT-trees
	ALCCandidate alc = GetALC(gp, start, start);

    //If ALC is an internal configuration of a CI, split the corresponding tree element accordingly
	if (alc.internalC)
		alc.treeElementIdx = tree.Split(alc.treeElementIdx, alc.q);

	//Check if there was a collision. If yes, then perform a turning movement in the other direction as well
	if (TurnAndExtend(tree, gp, alc, start, recentTCIIDs))
	{
		alc.angDist *= -1.0f;
		TurnAndExtend(tree, gp, alc, start, recentTCIIDs);
	}
}

bool Scene::TurnAndExtend(Tree &tree, Point &p, ALCCandidate &alc, bool headToGoal, vector<int> &recentTCIIDs)
{
	//Maximal collision-free RCI in the direction of the minimal turning amount to the GP
	ConfigInterval maxRCI;
	bool collision = TurnToPos(alc.q, p, sgn(alc.angDist), headToGoal, maxRCI);
	int idR = tree.AddElement(TreeElement(maxRCI), alc.treeElementIdx);

	//TCI extension of the new tree element added after the turn
	ConfigInterval forwardMaxTCI, backwardMaxTCI;
	TCI_Extension(tree.xtree[idR].q, forwardMaxTCI, backwardMaxTCI);

	//Add TCIs to the tree
	recentTCIIDs.push_back(tree.AddElement(TreeElement(forwardMaxTCI), idR));
	recentTCIIDs.push_back(tree.AddElement(TreeElement(backwardMaxTCI), idR));

	return collision;
}

ALCCandidate Scene::GetALC(Point &GP, bool start, bool preferredDir)
{
	Tree &tree = (start) ? startTree : goalTree;

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
	fixPrePathStartIdx = 0;
	fixPrePathGoalIdx = 0;

	recentTCIStartIDs.clear();
	recentTCIGoalIDs.clear();

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
				Line s0(envsx[i].ps[j], envsx[i].ps[((j + 1) % env_size)]);
				Config q0(robotShapeWorld.ps[k],q.phi);
				Point inter;

				if (Line::LineSegmentIntersection(Line(q0.p, q0.p + Point(cosf(q0.phi), sinf(q0.phi))), s0, inter))
				{
					float alfa = Point::atan2(inter, q0.p);

					if (fabs(Angle::Corrigate(alfa - q0.phi)) < EPS) //Forward collision possibility
					{
						forwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
					else if (fabs(Angle::Corrigate(-(PI - alfa) - q0.phi)) < EPS) //Backward collision possibility
					{
						backwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
				}

				//Collision check of obstacle corner points with robot polygon edges
				s0.a = robotShapeWorld.ps[k];
				s0.b = robotShapeWorld.ps[((k + 1) % rob_size)];
				q0.p = envsx[i].ps[j];				 

				if (Line::LineSegmentIntersection(Line(q0.p, q0.p + Point(cosf(q0.phi), sinf(q0.phi))), s0, inter))
				{
					float alfa = Point::atan2(q0.p, inter);				

					if (fabs(Angle::Corrigate(alfa - q0.phi)) < EPS) //Forward collision possibility
					{
						forwardPointDist.push_back(Point::Distance(inter, q0.p));
					}
					else if (fabs(Angle::Corrigate(-(PI - alfa) - q0.phi)) < EPS) //Backward collision possibility
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

/*
* TurnToPos: Let a polygonal robot turn in place to head or tail a given position
*
* INPUTS:
*   qStart     - Initial configuration
*   pos         - The "guiding" position
*   turndir     - The direction of the turn
*                 positive value: positive turning direction
*                 negative value: negative turning direction
*   headToGoal  - Determines the type of the desired end orientation 
*                 (there are two possibilities)
*                 positive value: end orientation points to the "guiding"
*                                 point (head to goal)
*                 negative value: end orientation points in the opposite
*                                 direction (tail to goal)
* OUTPUTS:
*   maxRCI     - Structure describing the maximal collision-free RCI
*                 (Rotational Configuration Interval) according to the
*                 inputs
*   maxRCI.type    - 1 (CI of rotation type)
*   maxRCI.q0      - starting configuration
*   maxRCI.q1      - end configuration
*   maxRCI.amount  - turning amount
*
*   return   - If a collision would occur before reching the desired
*                 orientation, this output parameter is set to 1, 
*                 else cleared to 0
*/
bool Scene::TurnToPos(Config qStart, Point pos, int turnDir, bool headToGoal, ConfigInterval &maxRCI)
{
	float thetaGoal, dThetaMax;

	//Determine the desired end orientation
	if (headToGoal)
		thetaGoal = Point::atan2(pos, qStart.p);	
	else
		thetaGoal = Point::atan2(qStart.p, pos);		

	//Determine the maximal turning amount
	dThetaMax = fabs(Angle::DirectedAngleDist(qStart.phi, thetaGoal, turnDir));

	//Transform the robot polygon to the world frame
	Polygon robotShapeWorld;
	transformRobotShape(robotShape, robotShapeWorld, qStart);

	//Collision check
	float dThetaAbs = numeric_limits<float>::infinity();
	float turnAmount;
	for (int i = 0; i < (int)envsx.size(); i++)
	{
		int env_size = envsx[i].ps.size();
		for (int j = 0; j < env_size; j++) //Traverse on obstacles's edges
		{
			int rob_size = robotShapeWorld.ps.size();
			for (int k = 0; k < rob_size; k++) //Traverse on robot's edges
			{
				//Collision check of robot polygon cornerpoints with obstacle edges	
				Point s0(envsx[i].ps[j]);							
				Point s1(envsx[i].ps[((j + 1) % env_size)]);
				Point p0(robotShapeWorld.ps[k]);

				turnAmount = maxCollFreeTurnAmountPointVsLineseg(p0, qStart.p, dThetaMax, turnDir, Line(s0, s1));
				if (turnAmount < dThetaAbs)
					dThetaAbs = turnAmount;

				//Collision check of obstacle cornerpoints with robot polygon edges
				s0 = robotShapeWorld.ps[k];
				s1 = robotShapeWorld.ps[((k + 1) % rob_size)];
				p0 = envsx[i].ps[j];				 

				turnAmount = maxCollFreeTurnAmountPointVsLineseg(p0, qStart.p, dThetaMax, -turnDir, Line(s0, s1));
				if (turnAmount < dThetaAbs)
					dThetaAbs = turnAmount;
			}
		}
	}

	bool collision = (fabs(dThetaAbs - dThetaMax) > EPS);

	dThetaAbs -=  min(dThetaAbs*0.01f, EPS);
	if (dThetaAbs < EPS)
		dThetaAbs = 0.0f;
	
	float dTheta = sgn(turnDir)*dThetaAbs;

	maxRCI.type = RotationCI;
	maxRCI.q0 = qStart;
	maxRCI.q1 = qStart;
	maxRCI.q1.phi = Angle::Corrigate(qStart.phi + dTheta);
	maxRCI.amount = dTheta;
	
	return collision;
}

float Scene::maxCollFreeTurnAmountPointVsLineseg(Point p, Point center, float dThetaMax, int turnDir, Line s1)
{
	float startOri = Angle::Corrigate(Point::atan2(p, center) + sgn(turnDir)*PI*0.5f);
	float radius = sgn(turnDir)*Point::Distance(p, center);

	Point res[2];
	int ret = circleSegLineSegIntersect(Config(p, startOri), sgn(turnDir)*dThetaMax, radius, s1, res);
	if (ret == 0) //No collision
	{
		return fabs(dThetaMax);
	}
	else if (ret == 1) //One collision
	{
		float endOri = Angle::Corrigate(Point::atan2(res[0], center) + sgn(turnDir)*PI*0.5f);
		return fabs(Angle::DirectedAngleDist(startOri, endOri, turnDir));
	}
	else //Two collision
	{
		float endOri1 = Angle::Corrigate(Point::atan2(res[0], center) + sgn(turnDir)*PI*0.5f);
		float dth1 = fabs(Angle::DirectedAngleDist(startOri, endOri1, turnDir));
		float endOri2 = Angle::Corrigate(Point::atan2(res[1], center) + sgn(turnDir)*PI*0.5f);
		float dth2 = fabs(Angle::DirectedAngleDist(startOri, endOri2, turnDir));

		return min(dth1, dth2);
	}
}

/*
* MergeTreesGetPath
* A function to seek merging possibilities of two RT-trees after one
* iteration of the RTR-planner algorithm. 
* Merging means here that intersection points are searched between TCIs of 
* different trees and if found, it is checked whether an RCI can be used to
* connect the intersecting TCIs at the intersection point.
* If there are more solutions, the simplest and shortest path is returned.
*
* INPUTS:
*   recentStartTCIIDs   - A array containing the ID-s of the 4 most
*                         recently added start TCI Tree element. -1 means invalid 
*                         TCI
*   recentGoalTCIIDs    - A array containing the ID-s of the 4 most
*                         recently added goal TCI Tree element. -1 means invalid 
*                         TCI
*
* OUTPUT:
*   pathCI              - A list of CI elements (alternating TCIs
*                         and RCIs) leading from the start configuration to
*                         the goal configuration. If the two trees could
*                         not be merged and no path was found, this output
*                         is an empty array
*/
bool Scene::MergeTreesGetPath()
{
	int minPathSize = numeric_limits<int>::max();
	float minPathLength = numeric_limits<float>::infinity();
	vector<ConfigInterval> pathCandidate;
	float pathCandidateLength;

	//Start tree with recent goal TCIs
	for (int i = 0; i < (int)recentTCIGoalIDs.size(); i++)
	{

		ConfigInterval mergeRCI;
		int ID = treeTCIMergeability(startTree, goalTree.xtree[recentTCIGoalIDs[i]].ci, mergeRCI);
		if (ID != -1)
		{
			pathCandidate = ObtainPath(ID, recentTCIGoalIDs[i], mergeRCI, pathCandidateLength);
			if (((int)pathCandidate.size() < minPathSize) || (((int)pathCandidate.size() == minPathSize) && (pathCandidateLength < minPathLength)))
			{
				minPathSize = pathCandidate.size();
				minPathLength = pathCandidateLength;
				pathCI = pathCandidate;
				
			}
		}
	}
	recentTCIGoalIDs.clear();

	//Goal tree with recent start TCIs
	for (int i = 0; i < (int)recentTCIStartIDs.size(); i++)
	{
		ConfigInterval mergeRCI;
		int ID = treeTCIMergeability(goalTree, startTree.xtree[recentTCIStartIDs[i]].ci, mergeRCI);
		if (ID != -1)
		{
			pathCandidate = ObtainPath(recentTCIStartIDs[i], ID, mergeRCI, pathCandidateLength);
			if (((int)pathCandidate.size() < minPathSize) || (((int)pathCandidate.size() == minPathSize) && (pathCandidateLength < minPathLength)))
			{
				minPathSize = pathCandidate.size();
				minPathLength = pathCandidateLength;
				pathCI = pathCandidate;
			}
		}
	}
	recentTCIStartIDs.clear();

	return (pathCI.size() > 0);
}

vector<ConfigInterval> Scene::ObtainPath(int startMergeID, int goalMergeID, ConfigInterval mergeRCI, float &pathLength)
{
	Point mergePos(mergeRCI.q0.p);

	//Split the corresponding tree elements at the intersection point
	int startTreeNewID = startTree.Split(startMergeID, Config(mergePos, startTree.xtree[startMergeID].ci.q0.phi));
	int goalTreeNewID = goalTree.Split(goalMergeID, Config(mergePos, goalTree.xtree[goalMergeID].ci.q0.phi));

	//1st part of the path: CIs from the start configuration to the newly added intersection point + merging RCI
	vector<ConfigInterval> path = startTree.PathFromRoot(startTreeNewID);
	reverse(path.begin(), path.end());
	path.push_back(mergeRCI);

	//2nd part of the path: CIs from the newly added intersection point to the goal configuration
	vector<ConfigInterval> path2 = goalTree.PathFromRoot(goalTreeNewID);
	path.insert(path.end(), path2.begin(), path2.end());

	//Determinate path length
	pathLength = 0.0f;
	for (vector<ConfigInterval>::iterator it = path.begin(); it != path.end(); ++it) //TODO: for(ConfigInterval& e : path) jó-e?
	{
		if (it->type == TranslationCI)
			pathLength += fabs(it->amount);
	}
	return path;
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
int Scene::circleSegLineSegIntersect(Config qStart, float dTheta, float radius, Line s1, Point res[2])
{
	Point center(qStart.p.x - radius*sinf(qStart.phi), qStart.p.y + radius*cosf(qStart.phi));
	Point resI[2];

	if (!Line::CircleLineIntersect(s1, fabs(radius), center, resI[0], resI[1]))
		return 0; 

	int interNum = 0;
	float startAngle = Angle::Corrigate(qStart.phi - sgn(radius)*PI*0.5f);

	for (int i = 0; i < 2; i++)
	{
		float checkAngle = Point::atan2(resI[i], center) - startAngle;

		if (sgn(checkAngle) != sgn(dTheta))
			checkAngle = sgn(dTheta)*(2*PI + sgn(dTheta)*checkAngle);

		if ((fabs(dTheta) >= fabs(checkAngle)) && s1.CheckPointInSegment(resI[i]))
			res[interNum++] = resI[i];
	}

	return interNum;
}


int Scene::treeTCIMergeability(Tree &tree, ConfigInterval TCI, ConfigInterval &mergingRCI)
{
	queue<int> idFIFO;

	//Seek through every tree elements in a breadth-first manner starting from the root element
	for (int i = 0; i < (int)tree.xtree[0].childrenIdx.size(); i++)
		idFIFO.push(tree.xtree[0].childrenIdx[i]); //Insert the children of the root first

	while (!idFIFO.empty())
	{
		int id = idFIFO.front();
		idFIFO.pop();

		if (tree.xtree[id].ci.type == RotationCI)
		{
			//RCI -> do nothing, only put its children to the queue
			for (int i = 0; i < (int)tree.xtree[id].childrenIdx.size(); i++)
				idFIFO.push(tree.xtree[id].childrenIdx[i]); //Insert the children 
		}
		else
		{
				//TCI -> check mergeability, if possible then break, else put children to the queue
			if (tciTCIMergeability(TCI, tree.xtree[id].ci, mergingRCI))
			{
				//Merging is possible -> return
				return id;
			}
			else
			{
				//Merging is not possible -> put children to the queue
				for (int i = 0; i < (int)tree.xtree[id].childrenIdx.size(); i++)
					idFIFO.push(tree.xtree[id].childrenIdx[i]); //Insert the children 
			}
		}
	}

	//Merging is not possible
	return -1;
}

bool Scene::tciTCIMergeability(ConfigInterval start, ConfigInterval end, ConfigInterval &mergingRCI)
{
	Point intersect;

	//Check if the two TCI-s are intersecting or not
	if (Line::SegmentSegmentItersection(Line(start.q0.p, start.q1.p), Line(end.q0.p, end.q1.p), intersect))
	{
		//If there is an intersection point then check if an RCI can be added to connect the two TCIs at this point
		Point localTargetPos;
			
		if (end.amount >= 0.0f)
			localTargetPos = end.q1.p; //Forward motion
		else
			localTargetPos = end.q0.p; //Backward motion

		Config c(intersect, start.q0.phi);
		float angDist = c.PointAngularDistance(localTargetPos, true);

		//Try to connect by the minimal RCI
		if (!TurnToPos(c, localTargetPos, sgn(angDist), true, mergingRCI))
			return true;

		//If the minimal RCI is colliding, then try the other turning
		if (!TurnToPos(c, localTargetPos, -sgn(angDist), true, mergingRCI))
			return true;
	}

	//No intersection
	return false;
}

Point Scene::GetGuidePoint(bool startPoint)
{
	default_random_engine generator;
	uniform_real_distribution<float> distribution(0.0f, 1.0f);

	float r = distribution(generator);

	if (r <= fixPathProbability)
	{
		return roadmap_node[static_cast<int>((distribution(generator)*roadmap_node.size()) + 0.5f)].p; //Mivel nincs round az MSVC2012-ben...
	}
	else if (r <= fixPathProbability + roadmapProbability)
	{
		if (startPoint)
			return fixPrePath[fixPrePathStartIdx++];
		else
			return fixPrePath[(int)fixPrePath.size() - 1 - fixPrePathGoalIdx++];
	}
	else
	{
		return Point(distribution(generator)*fieldXLength, distribution(generator)*fieldYLength);
	}
}