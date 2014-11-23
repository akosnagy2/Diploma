#include "Scene.h"
#include "CCSPlanner.h"

void PathPlanner::Scene::SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability, int randSeed)
{
	maxIteration = _maxIteration;
	fixPathProbability = _fixPathProbability;
	roadmapProbability = _roadmapProbability;
	if (randSeed ==  -1)
		generator.seed(rd());
	else
		generator.seed(randSeed);
}

void PathPlanner::Scene::SetCCSParameters(float _reversePentaltyFactor, float _useIntermediateS, float _insertCount, float _dx)
{
	reversePenaltyFactor = _reversePentaltyFactor;
	useIntermediateS = _useIntermediateS > 0.0f;
	insertCount = (int) _insertCount;
	dx = _dx;
}

bool PathPlanner::Scene::CCSPlanner()
{
	return CCSWrapper(*this, pathC);
}

void PathPlanner::Scene::SetFixPrePath(vector<Config> &fPath)
{
	for(vector<Config>::iterator it = fPath.begin(); it != fPath.end(); ++it)
		fixPrePath.push_back(it->p);
}